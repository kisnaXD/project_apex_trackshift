"""Nonblocking ownership and lifecycle helper for the EUFS simulation stack.

The Qt dashboard can call :meth:`tick` from its timer without waiting on a
child process.  Every process started here gets its own session, and only
those recorded process groups are ever signalled.  The helper intentionally
does not import ROS, so it remains usable by the dashboard before rclpy has
started spinning.
"""

from __future__ import annotations

import errno
import os
from pathlib import Path
import signal
import subprocess
import time
from dataclasses import dataclass, field
from typing import Dict, Mapping, Optional, Tuple


STACK_LOG = "/tmp/eufs_stack.log"
STOP_GRACE_S = 1.0
TERM_GRACE_S = 1.0
KILL_GRACE_S = 1.0
HELPER_NAMES = frozenset(("cota_lap.py", "drive_straight_10s.py"))


def _proc_stat(pid: int):
    """Return ``(pgrp, session, start_ticks, state)`` for a Linux process."""
    try:
        text = Path(f"/proc/{pid}/stat").read_text(encoding="utf-8")
    except (FileNotFoundError, PermissionError, OSError):
        return None
    closing = text.rfind(")")
    if closing < 0:
        return None
    fields = text[closing + 2 :].split()
    # After comm: state=0, ppid=1, pgrp=2, session=3, ... starttime=19.
    try:
        return int(fields[2]), int(fields[3]), int(fields[19]), fields[0]
    except (IndexError, ValueError):
        return None


def _proc_environ(pid: int):
    try:
        raw = Path(f"/proc/{pid}/environ").read_bytes()
    except (FileNotFoundError, PermissionError, OSError):
        return None
    values = {}
    for item in raw.split(b"\0"):
        if b"=" not in item:
            continue
        key, value = item.split(b"=", 1)
        values[key.decode("utf-8", "replace")] = value.decode("utf-8", "replace")
    return values


def _proc_argv(pid: int):
    try:
        raw = Path(f"/proc/{pid}/cmdline").read_bytes()
    except (FileNotFoundError, PermissionError, OSError):
        return ()
    return tuple(item.decode("utf-8", "replace") for item in raw.split(b"\0") if item)


@dataclass
class _OwnedGroup:
    process: subprocess.Popen
    label: str
    pid: int
    pgid: int
    session: int
    start_ticks: Optional[int]
    members: Dict[int, int] = field(default_factory=dict)


class StackManager:
    """Own the backend, viewers, and scoped helper cleanup state.

    ``start`` and ``shutdown`` only issue signals or spawn processes.  Call
    ``tick`` from the GUI event loop to advance shutdown escalation and queued
    restarts.
    """

    def __init__(self, env: Optional[Mapping[str, str]] = None):
        self._env = dict(os.environ if env is None else env)
        self._stack: Optional[_OwnedGroup] = None
        self._viewers: Dict[str, _OwnedGroup] = {}
        self._pending: Optional[Tuple[str, int]] = None
        self._config: Optional[Tuple[str, int]] = None
        self._state = "stopped"
        self._last_error: Optional[str] = None
        self._stop_phase: Optional[str] = None
        self._stop_deadline = 0.0
        self._helper_targets: Dict[int, int] = {}
        self._log_handle = None

    @property
    def state(self) -> str:
        return self._state

    @property
    def running(self) -> bool:
        return self._state in ("starting", "running", "restart_pending", "stopping")

    @property
    def track(self):
        config = self._pending or self._config
        return config[0] if config else None

    @property
    def cars(self):
        config = self._pending or self._config
        return config[1] if config else None

    @property
    def last_error(self) -> Optional[str]:
        return self._last_error

    @staticmethod
    def _validate(track, cars) -> Tuple[str, int]:
        track = str(track or "").strip()
        if not track or any(char not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_.-" for char in track):
            raise ValueError("track must be a nonempty simple name")
        try:
            cars = int(cars)
        except (TypeError, ValueError):
            raise ValueError("cars must be an integer from 1 to 20")
        if not 1 <= cars <= 20:
            raise ValueError("cars must be an integer from 1 to 20")
        return track, cars

    def _set_error(self, message):
        self._last_error = str(message)

    def _open_log(self):
        if self._log_handle is None or self._log_handle.closed:
            self._log_handle = open(STACK_LOG, "ab", buffering=0)
        return self._log_handle

    def _new_group(self, command, label, env=None):
        process = subprocess.Popen(
            command,
            env=self._env if env is None else env,
            stdout=self._open_log(),
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        stat = _proc_stat(process.pid)
        pgid = os.getpgid(process.pid)
        session = stat[1] if stat else pgid
        start_ticks = stat[2] if stat else None
        group = _OwnedGroup(
            process=process,
            label=label,
            pid=process.pid,
            pgid=pgid,
            session=session,
            start_ticks=start_ticks,
        )
        if start_ticks is not None:
            group.members[process.pid] = start_ticks
        return group

    def _viewer_env(self):
        env = dict(self._env)
        env.setdefault("DISPLAY", ":0")
        env["GAZEBO_MODEL_DATABASE_URI"] = ""
        env.setdefault("LIBGL_DRI3_DISABLE", "1")
        env.setdefault("QT_X11_NO_MITSHM", "1")
        resource_path = env.get("GAZEBO_RESOURCE_PATH", "")
        if "/usr/share/gazebo-11" not in resource_path.split(":"):
            env["GAZEBO_RESOURCE_PATH"] = ":".join(
                item for item in (resource_path, "/usr/share/gazebo-11") if item
            )
        return env

    def _launch_backend(self, config):
        track, cars = config
        command = [
            "ros2",
            "launch",
            "eufs_racecar",
            "load_car.launch.py",
            "show_dashboard:=false",
            f"track:={track}",
            f"cars:={cars}",
        ]
        try:
            self._stack = self._new_group(command, "ros2-launch")
        except (OSError, ValueError) as exc:
            self._stack = None
            self._state = "error"
            self._set_error(f"could not start EUFS stack: {exc}")
            return False
        self._config = config
        self._state = "running"
        self._last_error = None
        return True

    def start(self, track, cars):
        """Start a backend, or queue a changed configuration for restart."""
        try:
            config = self._validate(track, cars)
        except ValueError as exc:
            self._set_error(exc)
            return False

        if self._state in ("stopping", "restart_pending"):
            self._pending = config
            self._state = "restart_pending"
            return True

        if self._state in ("starting", "running") and self._stack is not None:
            if config == self._config and self._group_alive(self._stack):
                return True
            self._pending = config
            self._begin_stop(clear_pending=False)
            self._state = "restart_pending"
            return True

        if self._state == "error":
            self._state = "stopped"
        return self._launch_backend(config)

    def open_viewers(self, rviz_config):
        """Open owned gzclient and RViz processes against the running backend."""
        if self._stack is None or not self._group_alive(self._stack):
            self._set_error("cannot open viewers before the EUFS stack is running")
            return False
        commands = {
            "gzclient": ["gzclient"],
            "rviz2": [
                "rviz2", "-d", str(rviz_config),
                "--ros-args", "-p", "use_sim_time:=true", "-r", "__node:=rviz",
            ],
        }
        try:
            for name, command in commands.items():
                old = self._viewers.get(name)
                if old is not None and self._group_alive(old):
                    continue
                self._viewers[name] = self._new_group(command, name, self._viewer_env())
        except (OSError, ValueError) as exc:
            self._set_error(f"could not open viewers: {exc}")
            return False
        return True

    def shutdown(self):
        """Begin bounded asynchronous shutdown and cancel queued restarts."""
        self._pending = None
        if self._state in ("stopping", "restart_pending"):
            self._state = "stopping"
            return True
        self._begin_stop(clear_pending=True)
        return True

    def _begin_stop(self, clear_pending=True):
        if clear_pending:
            self._pending = None
        self._state = "stopping"
        self._stop_phase = "sigint"
        self._stop_deadline = time.monotonic() + STOP_GRACE_S
        self._refresh_groups()
        self._signal_groups(signal.SIGINT)
        self._signal_helpers(signal.SIGINT)

    def _groups(self):
        groups = []
        if self._stack is not None:
            groups.append(self._stack)
        groups.extend(self._viewers.values())
        return groups

    def _refresh_groups(self):
        for group in self._groups():
            self._refresh_group(group)

    @staticmethod
    def _refresh_group(group):
        for item in os.listdir("/proc"):
            if not item.isdigit():
                continue
            pid = int(item)
            stat = _proc_stat(pid)
            if stat is None or stat[3] in ("Z", "X") or stat[0] != group.pgid or stat[1] != group.session:
                continue
            group.members.setdefault(pid, stat[2])

    @staticmethod
    def _group_alive(group):
        # Reap an owned leader without waiting.  Otherwise a killed leader
        # can remain as a zombie in /proc and keep an already-empty group
        # looking alive to the nonblocking state machine.
        group.process.poll()
        StackManager._refresh_group(group)
        alive = False
        for pid, expected_start in list(group.members.items()):
            stat = _proc_stat(pid)
            if (
                stat is not None
                and stat[3] not in ("Z", "X")
                and stat[0] == group.pgid
                and stat[1] == group.session
                and stat[2] == expected_start
            ):
                alive = True
        return alive

    def _signal_groups(self, sig):
        for group in self._groups():
            if not self._group_alive(group):
                continue
            try:
                os.killpg(group.pgid, sig)
            except ProcessLookupError:
                continue
            except OSError as exc:
                if exc.errno != errno.ESRCH:
                    self._set_error(f"could not signal owned {group.label} group: {exc}")

    @staticmethod
    def _same_container(pid):
        try:
            return os.stat("/proc/self/root").st_ino == os.stat(f"/proc/{pid}/root").st_ino
        except (FileNotFoundError, PermissionError, OSError):
            return False

    def _same_context(self, pid):
        if not self._same_container(pid):
            return False
        values = _proc_environ(pid)
        if values is None:
            return False
        ros_domain = self._env.get("ROS_DOMAIN_ID", "0")
        helper_domain = values.get("ROS_DOMAIN_ID", "0")
        master = self._env.get("GAZEBO_MASTER_URI", "http://localhost:11345")
        helper_master = values.get("GAZEBO_MASTER_URI", "http://localhost:11345")
        return ros_domain == helper_domain and master == helper_master

    def _discover_helpers(self):
        found = {}
        for item in os.listdir("/proc"):
            if not item.isdigit():
                continue
            pid = int(item)
            if pid == os.getpid():
                continue
            argv = _proc_argv(pid)
            if not argv or not Path(argv[0]).name.startswith("python"):
                continue
            script = None
            args = list(argv[1:])
            while args:
                arg = args.pop(0)
                if arg in ("-c", "-m"):
                    script = None
                    break
                if arg.startswith("-"):
                    # Python's common interpreter switches are flag-only;
                    # options carrying a value are skipped with that value.
                    if arg in ("-W", "-X") and args:
                        args.pop(0)
                    continue
                script = arg
                break
            if script is None or Path(script).name not in HELPER_NAMES:
                continue
            stat = _proc_stat(pid)
            if stat is None or stat[3] in ("Z", "X") or not self._same_context(pid):
                continue
            found[pid] = stat[2]
        self._helper_targets.update(found)
        for pid, expected_start in list(self._helper_targets.items()):
            stat = _proc_stat(pid)
            if stat is None or stat[3] in ("Z", "X") or stat[2] != expected_start:
                self._helper_targets.pop(pid, None)
        return dict(self._helper_targets)

    def _signal_helpers(self, sig):
        for pid, expected_start in self._discover_helpers().items():
            stat = _proc_stat(pid)
            if stat is None or stat[3] in ("Z", "X") or stat[2] != expected_start:
                continue
            try:
                os.kill(pid, sig)
            except ProcessLookupError:
                self._helper_targets.pop(pid, None)
            except OSError as exc:
                if exc.errno != errno.ESRCH:
                    self._set_error(f"could not signal helper {pid}: {exc}")

    def _all_stopped(self):
        self._refresh_groups()
        groups_alive = any(self._group_alive(group) for group in self._groups())
        helpers_alive = bool(self._discover_helpers())
        return not groups_alive and not helpers_alive

    def _finish_stop(self):
        if self._log_handle is not None and not self._log_handle.closed:
            self._log_handle.close()
        self._log_handle = None
        self._stack = None
        self._viewers.clear()
        pending = self._pending
        self._pending = None
        self._stop_phase = None
        self._state = "stopped"
        if pending is not None:
            self._launch_backend(pending)

    def tick(self):
        """Advance lifecycle state without waiting for any process."""
        now = time.monotonic()
        if self._state in ("starting", "running"):
            if self._stack is not None and not self._group_alive(self._stack):
                self._set_error("EUFS stack exited unexpectedly")
                self._begin_stop(clear_pending=True)
            return self._state
        if self._state not in ("stopping", "restart_pending"):
            return self._state

        self._signal_helpers(signal.SIGINT if self._stop_phase == "sigint" else signal.SIGTERM if self._stop_phase == "sigterm" else signal.SIGKILL)
        if self._all_stopped():
            self._finish_stop()
            return self._state
        if now < self._stop_deadline:
            return self._state

        if self._stop_phase == "sigint":
            self._stop_phase = "sigterm"
            self._stop_deadline = now + TERM_GRACE_S
            self._signal_groups(signal.SIGTERM)
            self._signal_helpers(signal.SIGTERM)
        elif self._stop_phase == "sigterm":
            self._stop_phase = "sigkill"
            self._stop_deadline = now + KILL_GRACE_S
            self._signal_groups(signal.SIGKILL)
            self._signal_helpers(signal.SIGKILL)
        else:
            # Keep the state stopping until the owned processes disappear;
            # this prevents a queued restart from inheriting leaked commands.
            self._stop_deadline = now + KILL_GRACE_S
            self._signal_groups(signal.SIGKILL)
            self._signal_helpers(signal.SIGKILL)
        return self._state
