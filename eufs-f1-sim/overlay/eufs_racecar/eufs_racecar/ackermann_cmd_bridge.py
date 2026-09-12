"""EUFS-shaped Ackermann (accel + steer) → Twist for this racecar.

Stock EUFS rqt publishes ackermann_msgs/AckermannDriveStamped on /cmd with
acceleration and steering_angle. This car's Gazebo stack consumes Twist on
cmd_vel (gazebo_ros_ackermann_drive). The bridge is the
translation so a 10s throttle can command the real EUFS fields.
"""

from __future__ import annotations

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist
from rclpy.node import Node
import rclpy
import time

from .ackermann_cmd_logic import SpeedController


class AckermannCmdBridge(Node):
    def __init__(self):
        super().__init__('ackermann_cmd_bridge')
        self.declare_parameter('namespace', 'eufs')
        self.declare_parameter('max_speed_mps', 80.0)
        self.declare_parameter('max_steer_rad', 0.6458)
        self.declare_parameter('accel_only_speed_mps', 2.0)
        self.declare_parameter('command_timeout_s', 0.5)
        self.declare_parameter('stale_decel_mps2', 2.0)
        namespace = str(self.get_parameter('namespace').value).strip('/')
        ns = f'/{namespace}' if namespace else ''
        self.max_speed = float(self.get_parameter('max_speed_mps').value)
        self.max_steer = float(self.get_parameter('max_steer_rad').value)
        self._controller = SpeedController(
            max_speed_mps=self.max_speed,
            accel_only_speed_mps=float(self.get_parameter('accel_only_speed_mps').value),
            command_timeout_s=float(self.get_parameter('command_timeout_s').value),
            stale_decel_mps2=float(self.get_parameter('stale_decel_mps2').value),
        )
        self._steer = 0.0

        self.create_subscription(
            AckermannDriveStamped, f'{ns}/cmd' if ns else '/cmd', self._on_ack, 10,
        )
        self.pub = self.create_publisher(Twist, f'{ns}/cmd_vel' if ns else '/cmd_vel', 10)
        self.create_timer(0.05, self._tick)
        self.get_logger().info(
            f'Ackermann {ns}/cmd → Twist {ns}/cmd_vel (speed+accel, steer=angular.z)'
        )

    def _on_ack(self, msg: AckermannDriveStamped):
        self._steer = max(-self.max_steer, min(self.max_steer, float(msg.drive.steering_angle)))
        now = time.monotonic()
        self._controller.accept(msg.drive.speed, msg.drive.acceleration, now)

    def _tick(self):
        now = time.monotonic()
        speed = self._controller.update(now)
        if speed is None:
            return
        twist = Twist()
        twist.linear.x = float(speed)
        twist.angular.z = float(self._steer)
        self.pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = AckermannCmdBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
