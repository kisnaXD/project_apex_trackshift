import math
import pathlib
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).parents[1] / 'overlay' / 'eufs_racecar'))

import rclpy
from nav_msgs.msg import Odometry

from eufs_racecar.odom_tf_publisher import OdomTfPublisher


class _Collector:
    def __init__(self):
        self.messages = []

    def sendTransform(self, messages):
        self.messages.append(messages)


def _yaw_from_quat(q):
    return 2.0 * math.atan2(q.z, q.w)


class OdomTfPublisherTest(unittest.TestCase):
    def _node(self, relative):
        rclpy.init(args=[
            'test_odom_tf_publisher', '--ros-args',
            '-p', 'namespace:=eufs', '-p', 'x:=10.0', '-p', 'y:=20.0',
            '-p', 'z:=0.1', '-p', f'yaw:={math.pi / 2}',
            '-p', f'odom_relative:={str(relative).lower()}',
        ])
        node = OdomTfPublisher()
        node._broadcaster = _Collector()
        return node

    def test_relative_messages_preserve_spawn_map_and_local_odom(self):
        node = self._node(True)
        try:
            node._tick()
            before = node._broadcaster.messages[-1]
            self.assertEqual((before[0].transform.translation.x,
                              before[0].transform.translation.y), (10.0, 20.0))
            self.assertAlmostEqual(before[1].transform.translation.z, 0.1)
            self.assertAlmostEqual(before[0].transform.translation.z, 0.0)
            self.assertAlmostEqual(_yaw_from_quat(before[1].transform.rotation), 0.0)
            self.assertAlmostEqual(_yaw_from_quat(before[0].transform.rotation), math.pi / 2)

            msg = Odometry()
            msg.pose.pose.position.x = 2.0
            msg.pose.pose.position.z = 0.1
            msg.pose.pose.orientation.w = 1.0
            node._on_odom(msg)
            node._tick()
            after = node._broadcaster.messages[-1]
            # map -> odom -> base_link composes to world (10, 22, 0.1).
            map_yaw = _yaw_from_quat(after[0].transform.rotation)
            world_x = after[0].transform.translation.x + math.cos(map_yaw) * after[1].transform.translation.x - math.sin(map_yaw) * after[1].transform.translation.y
            world_y = after[0].transform.translation.y + math.sin(map_yaw) * after[1].transform.translation.x + math.cos(map_yaw) * after[1].transform.translation.y
            self.assertAlmostEqual(world_x, 10.0)
            self.assertAlmostEqual(world_y, 22.0)
            self.assertAlmostEqual(after[1].transform.translation.z, 0.1)
            self.assertAlmostEqual(after[0].transform.translation.z, 0.0)
            self.assertAlmostEqual(_yaw_from_quat(after[0].transform.rotation), math.pi / 2)
        finally:
            node.destroy_node()
            rclpy.shutdown()

    def test_ackermann_initial_pose_is_world_pose(self):
        node = self._node(False)
        try:
            node._tick()
            transforms = node._broadcaster.messages[-1]
            base = transforms[1]
            self.assertAlmostEqual(base.transform.translation.x, 10.0)
            self.assertAlmostEqual(base.transform.translation.y, 20.0)
            self.assertAlmostEqual(base.transform.translation.z, 0.1)
            self.assertAlmostEqual(_yaw_from_quat(base.transform.rotation), math.pi / 2)
        finally:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
