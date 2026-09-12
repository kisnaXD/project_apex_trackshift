"""Offline tests for /eufs/cmd self-echo ownership on Humble."""
from __future__ import annotations

import copy
import struct
from collections import deque

from ackermann_msgs.msg import AckermannDriveStamped
from builtin_interfaces.msg import Time

from scripts.cota_lap import Lap


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(copy.deepcopy(message))


class _Now:
    def __init__(self, sec, nanosec):
        self.stamp = Time(sec=sec, nanosec=nanosec)

    def to_msg(self):
        return self.stamp


class _Clock:
    def __init__(self):
        self.sec = 10
        self.nanosec = 1

    def now(self):
        return _Now(self.sec, self.nanosec)


class _Logger:
    def warning(self, _message):
        pass


class _EchoHarness:
    _command_fingerprint = staticmethod(Lap._command_fingerprint)

    def __init__(self):
        self.cmd_pub = _Publisher()
        self.external_override = False
        self.external_node = ""
        self.failed = None
        self.own_command_gids = set()
        self._own_command_fingerprints = deque(maxlen=1024)
        self._own_command_fingerprint_set = set()
        self.last_steer = 0.0
        self._clock = _Clock()

    def get_clock(self):
        return self._clock

    def get_name(self):
        return "eufs_cota_lap"

    def get_logger(self):
        return _Logger()

    def _refresh_own_command_gids(self):
        pass


def test_wire_float32_quantization_has_same_fingerprint():
    first = AckermannDriveStamped()
    first.header.stamp = Time(sec=4, nanosec=8)
    first.drive.steering_angle = 0.123456789
    first.drive.speed = 7.654321987
    first.drive.acceleration = -1.19999991
    second = copy.deepcopy(first)
    second.drive.steering_angle = struct.unpack("<f", struct.pack("<f", first.drive.steering_angle))[0]
    second.drive.speed = struct.unpack("<f", struct.pack("<f", first.drive.speed))[0]
    second.drive.acceleration = struct.unpack("<f", struct.pack("<f", first.drive.acceleration))[0]
    assert Lap._command_fingerprint(first) == Lap._command_fingerprint(second)


def test_delayed_own_echo_survives_newer_command():
    harness = _EchoHarness()
    Lap._command(harness, 0.1, 4.0, 1.0)
    old_message = copy.deepcopy(harness.cmd_pub.messages[-1])
    harness._clock.sec += 1
    Lap._command(harness, -0.2, 2.0, -1.2)
    Lap._on_external_command(harness, old_message)
    assert harness.external_override is False
    assert harness.failed is None


def test_nonmatching_command_yields_authority():
    harness = _EchoHarness()
    Lap._command(harness, 0.1, 4.0, 1.0)
    external = AckermannDriveStamped()
    external.header.stamp = Time(sec=99, nanosec=1)
    external.drive.speed = 4.0
    external.drive.acceleration = 1.0
    Lap._on_external_command(harness, external)
    assert harness.external_override is True
    assert harness.failed == "external command override"
