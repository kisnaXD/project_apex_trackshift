"""Broadcast map→odom→base_link and wheel joints so RViz RobotModel works paused.

Gazebo's ackermann plugin does not publish odom TF until physics steps, and
rclpy sim-time timers do not fire while /clock is frozen. This node uses a
wall-clock timer and stamps transforms with /clock (0 while paused).
"""

from __future__ import annotations

import math

from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
import rclpy

WHEEL_JOINTS = (
    'left_front_wheel_joint',
    'right_front_wheel_joint',
    'left_rear_wheel_joint',
    'right_rear_wheel_joint',
    'left_steering_hinge_joint',
    'right_steering_hinge_joint',
)


def _yaw_to_quat(yaw: float):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class OdomTfPublisher(Node):
    def __init__(self):
        super().__init__('odom_tf_publisher')
        self.declare_parameter('namespace', 'eufs')
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('z', 0.08)
        self.declare_parameter('yaw', 0.0)
        namespace = str(self.get_parameter('namespace').value).strip('/')
        ns = f'/{namespace}' if namespace else ''
        self._x = float(self.get_parameter('x').value)
        self._y = float(self.get_parameter('y').value)
        self._z = float(self.get_parameter('z').value)
        self._qx, self._qy, self._qz, self._qw = _yaw_to_quat(
            float(self.get_parameter('yaw').value)
        )
        self._sim_stamp = None
        self._broadcaster = TransformBroadcaster(self)
        self._joint_pub = self.create_publisher(
            JointState, f'{ns}/joint_states' if ns else '/joint_states', 10,
        )
        clock_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Clock, '/clock', self._on_clock, clock_qos)
        self.create_subscription(
            Odometry, f'{ns}/odom' if ns else '/odom', self._on_odom, 10,
        )
        self.create_timer(0.05, self._tick)
        self.get_logger().info(
            f'map→odom→base_link spawn ({self._x:.3f}, {self._y:.3f}, {self._z:.3f})'
        )

    def _on_clock(self, msg: Clock):
        self._sim_stamp = msg.clock

    def _on_odom(self, msg: Odometry):
        pose = msg.pose.pose
        self._x = pose.position.x
        self._y = pose.position.y
        self._z = pose.position.z
        self._qx = pose.orientation.x
        self._qy = pose.orientation.y
        self._qz = pose.orientation.z
        self._qw = pose.orientation.w

    def _stamp(self):
        if self._sim_stamp is not None:
            return self._sim_stamp
        return Time()

    def _tick(self):
        now = self._stamp()
        map_tf = TransformStamped()
        map_tf.header.stamp = now
        map_tf.header.frame_id = 'map'
        map_tf.child_frame_id = 'odom'
        map_tf.transform.rotation.w = 1.0
        base_tf = TransformStamped()
        base_tf.header.stamp = now
        base_tf.header.frame_id = 'odom'
        base_tf.child_frame_id = 'base_link'
        base_tf.transform.translation.x = self._x
        base_tf.transform.translation.y = self._y
        base_tf.transform.translation.z = self._z
        base_tf.transform.rotation.x = self._qx
        base_tf.transform.rotation.y = self._qy
        base_tf.transform.rotation.z = self._qz
        base_tf.transform.rotation.w = self._qw
        self._broadcaster.sendTransform([map_tf, base_tf])

        joints = JointState()
        joints.header.stamp = now
        joints.name = list(WHEEL_JOINTS)
        joints.position = [0.0] * len(WHEEL_JOINTS)
        joints.velocity = [0.0] * len(WHEEL_JOINTS)
        self._joint_pub.publish(joints)


def main(args=None):
    rclpy.init(args=args)
    node = OdomTfPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
