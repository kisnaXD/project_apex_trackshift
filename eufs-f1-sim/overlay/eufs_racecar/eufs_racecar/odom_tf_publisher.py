"""Broadcast map→odom→base_link while Gazebo is paused.

Gazebo's ackermann plugin does not publish odom TF until physics steps, and
rclpy sim-time timers do not fire while /clock is frozen. This node uses a
wall-clock timer and stamps transforms with /clock (0 while paused).
Wheel joints come from gazebo_ros_joint_state_publisher so RViz can see them roll.
"""

from __future__ import annotations

import math

from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from tf2_ros import TransformBroadcaster
import rclpy
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
        self.declare_parameter('odom_relative', False)
        self.declare_parameter('frame_prefix', '')
        namespace = str(self.get_parameter('namespace').value).strip('/')
        ns = f'/{namespace}' if namespace else ''
        self._spawn_x = float(self.get_parameter('x').value)
        self._spawn_y = float(self.get_parameter('y').value)
        self._spawn_z = float(self.get_parameter('z').value)
        self._spawn_yaw = float(self.get_parameter('yaw').value)
        self._odom_relative = bool(self.get_parameter('odom_relative').value)
        self._frame_prefix = str(self.get_parameter('frame_prefix').value)
        if self._odom_relative:
            # DynamicBicycle odometry is local to the spawn pose.  Preserve
            # that local frame until odometry arrives; never rebase the first
            # sample or apply the spawn offset twice.
            self._x, self._y, self._z = 0.0, 0.0, self._spawn_z
            self._qx, self._qy, self._qz, self._qw = _yaw_to_quat(0.0)
        else:
            self._x, self._y, self._z = self._spawn_x, self._spawn_y, self._spawn_z
            self._qx, self._qy, self._qz, self._qw = _yaw_to_quat(self._spawn_yaw)
        self._spawn_qx, self._spawn_qy, self._spawn_qz, self._spawn_qw = _yaw_to_quat(self._spawn_yaw)
        self._sim_stamp = None
        self._broadcaster = TransformBroadcaster(self)
        clock_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Clock, '/clock', self._on_clock, clock_qos)
        self.create_subscription(
            Odometry, f'{ns}/odom' if ns else '/odom', self._on_odom, 10,
        )
        self.create_timer(0.05, self._tick)
        self.get_logger().info(
            f'map→odom→base_link spawn ({self._spawn_x:.3f}, {self._spawn_y:.3f}, {self._spawn_z:.3f})'
        )

    def _on_clock(self, msg: Clock):
        self._sim_stamp = msg.clock

    def _on_odom(self, msg: Odometry):
        pose = msg.pose.pose
        if self._odom_relative:
            self._x, self._y, self._z = pose.position.x, pose.position.y, pose.position.z
            self._qx, self._qy, self._qz, self._qw = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        else:
            self._x, self._y, self._z = pose.position.x, pose.position.y, pose.position.z
            self._qx, self._qy, self._qz, self._qw = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)

    def _stamp(self):
        if self._sim_stamp is not None:
            return self._sim_stamp
        return Time()

    def _tick(self):
        now = self._stamp()
        map_tf = TransformStamped()
        map_tf.header.stamp = now
        map_tf.header.frame_id = 'map'
        map_tf.child_frame_id = f'{self._frame_prefix}odom'
        if self._odom_relative:
            map_tf.transform.translation.x = self._spawn_x
            map_tf.transform.translation.y = self._spawn_y
            map_tf.transform.rotation.z = self._spawn_qz
            map_tf.transform.rotation.w = self._spawn_qw
        else:
            map_tf.transform.rotation.w = 1.0
        base_tf = TransformStamped()
        base_tf.header.stamp = now
        base_tf.header.frame_id = f'{self._frame_prefix}odom'
        base_tf.child_frame_id = f'{self._frame_prefix}base_link'
        base_tf.transform.translation.x = self._x
        base_tf.transform.translation.y = self._y
        base_tf.transform.translation.z = self._z
        base_tf.transform.rotation.x = self._qx
        base_tf.transform.rotation.y = self._qy
        base_tf.transform.rotation.z = self._qz
        base_tf.transform.rotation.w = self._qw
        self._broadcaster.sendTransform([map_tf, base_tf])


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
