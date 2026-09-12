import unittest

from eufs_racecar.ackermann_cmd_logic import SpeedController


class SpeedControllerTest(unittest.TestCase):
    def test_positive_acceleration_only_uses_bounded_target(self):
        c = SpeedController(max_speed_mps=80.0, accel_only_speed_mps=2.0, command_timeout_s=10.0)
        c.accept(0.0, 0.5, 0.0)
        self.assertAlmostEqual(c.update(1.0), 0.5)
        self.assertAlmostEqual(c.update(5.0), 2.0)

    def test_negative_acceleration_at_zero_speed_brakes_to_zero(self):
        c = SpeedController(command_timeout_s=10.0)
        c.speed = 1.5
        c.accept(0.0, -0.5, 0.0)
        self.assertAlmostEqual(c.update(1.0), 1.0)
        self.assertAlmostEqual(c.update(3.0), 0.0)

    def test_stale_command_brakes_instead_of_holding(self):
        c = SpeedController(command_timeout_s=0.5, stale_decel_mps2=2.0)
        c.accept(2.0, 0.0, 0.0)
        c.speed = 2.0
        self.assertAlmostEqual(c.update(1.0), 0.0)

    def test_backwards_clock_jump_does_not_integrate(self):
        c = SpeedController()
        c.accept(2.0, 0.5, 10.0)
        c.update(10.1)
        before = c.speed
        self.assertAlmostEqual(c.update(5.0), before)


if __name__ == '__main__':
    unittest.main()
