import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest


PACKAGE_ROOT = Path(__file__).parents[1] / "overlay" / "eufs_racecar"
sys.path.insert(0, str(PACKAGE_ROOT))

from eufs_racecar import stack_manager  # noqa: E402
from eufs_racecar.stack_manager import StackManager  # noqa: E402


class StackManagerTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.bin = self.root / "bin"
        self.bin.mkdir()
        fake = "#!/bin/sh\nexec sleep 60\n"
        for name in ("ros2", "gzclient", "rviz2"):
            path = self.bin / name
            path.write_text(fake, encoding="utf-8")
            path.chmod(0o755)
        self.env = dict(os.environ)
        self.env["PATH"] = f"{self.bin}:{self.env.get('PATH', '')}"
        self.env["ROS_DOMAIN_ID"] = "61"
        self.env["GAZEBO_MASTER_URI"] = "http://127.0.0.1:11345"
        self._old_graces = (
            stack_manager.STOP_GRACE_S,
            stack_manager.TERM_GRACE_S,
            stack_manager.KILL_GRACE_S,
        )
        stack_manager.STOP_GRACE_S = 0.05
        stack_manager.TERM_GRACE_S = 0.05
        stack_manager.KILL_GRACE_S = 0.05

    def tearDown(self):
        stack_manager.STOP_GRACE_S, stack_manager.TERM_GRACE_S, stack_manager.KILL_GRACE_S = self._old_graces
        self.temp.cleanup()

    @staticmethod
    def _finish(manager, timeout=3.0):
        deadline = time.monotonic() + timeout
        while manager.state != "stopped" and time.monotonic() < deadline:
            manager.tick()
            time.sleep(0.01)
        return manager.state

    def test_idempotent_start_and_queued_restart(self):
        manager = StackManager(self.env)
        self.assertTrue(manager.start("cota", 1))
        first_pid = manager._stack.pid
        self.assertTrue(manager.start("cota", 1))
        self.assertEqual(first_pid, manager._stack.pid)

        self.assertTrue(manager.start("small_track", 2))
        self.assertEqual(manager.state, "restart_pending")
        self.assertEqual((manager.track, manager.cars), ("small_track", 2))
        self.assertEqual(self._finish(manager), "running")
        self.assertNotEqual(first_pid, manager._stack.pid)
        self.assertEqual((manager.track, manager.cars), ("small_track", 2))
        manager.shutdown()
        self.assertEqual(self._finish(manager), "stopped")

    def test_viewers_are_owned_and_shutdown_is_bounded(self):
        manager = StackManager(self.env)
        viewer_env = manager._viewer_env()
        self.assertIn("/usr/share/gazebo-11", viewer_env["GAZEBO_RESOURCE_PATH"].split(":"))
        self.assertTrue(manager.start("cota", 1))
        self.assertTrue(manager.open_viewers("/tmp/eufs.rviz"))
        self.assertEqual(set(manager._viewers), {"gzclient", "rviz2"})
        pids = [group.pid for group in manager._viewers.values()]
        manager.shutdown()
        self.assertEqual(self._finish(manager), "stopped")
        for pid in pids:
            with self.assertRaises(ProcessLookupError):
                os.kill(pid, 0)

    def test_start_during_shutdown_queues_even_same_configuration(self):
        manager = StackManager(self.env)
        self.assertTrue(manager.start("cota", 1))
        manager.shutdown()
        self.assertTrue(manager.start("cota", 1))
        self.assertEqual(manager.state, "restart_pending")
        self.assertEqual(self._finish(manager), "running")
        manager.shutdown()
        self.assertEqual(self._finish(manager), "stopped")

    def test_matching_lap_helper_is_cleaned_but_other_domain_survives(self):
        helper = self.root / "cota_lap.py"
        helper.write_text(
            "import signal, time\n"
            "signal.signal(signal.SIGINT, lambda *_: raise_exit())\n"
            "def raise_exit(): raise SystemExit(0)\n"
            "while True: time.sleep(1)\n",
            encoding="utf-8",
        )
        matching = subprocess.Popen([sys.executable, str(helper)], env=self.env)
        other_env = dict(self.env)
        other_env["ROS_DOMAIN_ID"] = "62"
        other = subprocess.Popen([sys.executable, str(helper)], env=other_env)
        manager = StackManager(self.env)
        self.assertTrue(manager.start("cota", 1))
        manager.shutdown()
        self.assertEqual(self._finish(manager), "stopped")
        matching.wait(timeout=1.0)
        self.assertIsNone(other.poll())
        other.send_signal(signal.SIGTERM)
        other.wait(timeout=1.0)

    def test_validation_rejects_out_of_range_cars(self):
        manager = StackManager(self.env)
        self.assertFalse(manager.start("cota", 0))
        self.assertIn("1 to 20", manager.last_error)
        self.assertFalse(manager.start("bad track", 1))


if __name__ == "__main__":
    unittest.main()
