"""EUFS-shaped Ackermann (accel + steer) → Twist for this racecar.

Stock EUFS rqt publishes ackermann_msgs/AckermannDriveStamped on /cmd with
acceleration and steering_angle. This car's Gazebo stack consumes Twist on
cmd_vel (energy-aware gate → gazebo_ros_ackermann_drive). The bridge is the
translation so a 10s throttle can command the real EUFS fields.
"""

from __future__ import annotations

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist
from rclpy.node import Node
import rclpy


class AckermannCmdBridge(Node):
    def __init__(self):
        super().__init__('ackermann_cmd_bridge')
        self.declare_parameter('namespace', 'eufs')
        self.declare_parameter('max_speed_mps', 80.0)
        self.declare_parameter('max_steer_rad', 0.6458)
        namespace = str(self.get_parameter('namespace').value).strip('/')
        ns = f'/{namespace}' if namespace else ''
        self.max_speed = float(self.get_parameter('max_speed_mps').value)
        self.max_steer = float(self.get_parameter('max_steer_rad').value)
        self._speed = 0.0
        self._target_speed = 0.0
        self._steer = 0.0
        self._accel = 0.0
        self._have_cmd = False
        self._last_time = None

        self.create_subscription(
            AckermannDriveStamped, f'{ns}/cmd' if ns else '/cmd', self._on_ack, 10,
        )
        self.pub = self.create_publisher(Twist, f'{ns}/cmd_vel' if ns else '/cmd_vel', 10)
        self.create_timer(0.05, self._tick)
        self.get_logger().info(
            f'Ackermann {ns}/cmd → Twist {ns}/cmd_vel (speed+accel, steer=angular.z)'
        )

    def _on_ack(self, msg: AckermannDriveStamped):
        self._have_cmd = True
        self._steer = max(-self.max_steer, min(self.max_steer, float(msg.drive.steering_angle)))
        self._accel = float(msg.drive.acceleration)
        # EUFS velocity-mode: drive.speed is the target; accel ramps toward it.
        # If speed is left at 0 and accel is set, integrate speed (rqt-style).
        speed = float(msg.drive.speed)
        if abs(speed) > 1e-3:
            self._target_speed = max(-self.max_speed, min(self.max_speed, speed))
        elif abs(self._accel) > 1e-6:
            self._target_speed = self.max_speed if self._accel > 0.0 else -self.max_speed
        else:
            self._target_speed = 0.0

    def _tick(self):
        now = self.get_clock().now()
        if self._last_time is None:
            self._last_time = now
            return
        dt = (now - self._last_time).nanoseconds * 1e-9
        self._last_time = now
        if dt <= 0.0 or dt > 0.5:
            return
        if not self._have_cmd:
            return
        target = getattr(self, '_target_speed', 0.0)
        if abs(self._accel) > 1e-6:
            step = self._accel * dt
            if target >= self._speed:
                self._speed = min(target, self._speed + abs(step))
            else:
                self._speed = max(target, self._speed - abs(step))
        else:
            self._speed = target
        self._speed = max(-self.max_speed, min(self.max_speed, self._speed))
        twist = Twist()
        twist.linear.x = float(self._speed)
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
