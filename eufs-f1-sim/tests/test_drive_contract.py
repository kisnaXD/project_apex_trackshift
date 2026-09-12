import pathlib
import sys
import unittest


PACKAGE_ROOT = pathlib.Path(__file__).parents[1] / 'overlay' / 'eufs_racecar'
sys.path.insert(0, str(PACKAGE_ROOT))

from eufs_racecar.drive_contract import (  # noqa: E402
    DURATION_S,
    MAX_ACCEL_MPS2,
    MAX_SPEED_MPS,
    SimTimedDrive,
    braking_acceleration,
    clamp_command,
    forward_feedback_acceleration,
)


class DriveContractTest(unittest.TestCase):
    def test_speed_and_acceleration_are_capped(self):
        self.assertEqual(clamp_command(99.0, 99.0), (MAX_SPEED_MPS, MAX_ACCEL_MPS2))
        self.assertEqual(clamp_command(-99.0, -99.0), (-MAX_SPEED_MPS, -MAX_ACCEL_MPS2))

    def test_zero_acceleration_is_not_used_for_braking(self):
        self.assertEqual(braking_acceleration(1.0), -MAX_ACCEL_MPS2)
        self.assertEqual(braking_acceleration(-1.0), MAX_ACCEL_MPS2)
        self.assertEqual(braking_acceleration(0.04), 0.0)
        self.assertEqual(braking_acceleration(-0.04), 0.0)

    def test_forward_feedback_tapers_and_brakes_at_target(self):
        self.assertEqual(forward_feedback_acceleration(0.0), MAX_ACCEL_MPS2)
        self.assertEqual(forward_feedback_acceleration(MAX_SPEED_MPS), 0.0)
        self.assertEqual(forward_feedback_acceleration(MAX_SPEED_MPS + 0.2), -MAX_ACCEL_MPS2)

    def test_sim_duration_clock_stall_and_wall_timeout(self):
        drive = SimTimedDrive()
        drive.arm(10.0)
        drive.started_wall = 0.0
        self.assertEqual(drive.state(10.0, 0.0), 'waiting')
        self.assertEqual(drive.state(10.1, 1.0), 'active')
        self.assertEqual(drive.state(10.1, 59.9), 'active')
        self.assertEqual(drive.state(10.1, 60.0), 'timeout')

        drive.arm(10.0)
        drive.started_wall = 0.0
        self.assertEqual(drive.state(10.1, 1.0), 'active')
        self.assertEqual(drive.state(10.1 + DURATION_S, 2.0), 'done')

    def test_clock_and_telemetry_readiness_wait(self):
        drive = SimTimedDrive()
        drive.arm(None)
        drive.started_wall = 0.0
        self.assertEqual(drive.state(None, 1.0), 'waiting')
        self.assertEqual(drive.state(0.0, 2.0), 'waiting')
        self.assertEqual(drive.state(0.0, 60.0), 'timeout')

    def test_cancel_and_second_run_reset(self):
        drive = SimTimedDrive()
        drive.arm(0.0)
        drive.cancel()
        self.assertEqual(drive.state(1.0, 1.0), 'cancelled')
        drive.arm(20.0)
        self.assertEqual(drive.state(20.1, 1.0), 'active')


if __name__ == '__main__':
    unittest.main()
