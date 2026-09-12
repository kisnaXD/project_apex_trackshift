"""Publish the EUFS SDF cone layout using stock RViz MarkerArray messages.

The lean container does not include eufs_rviz_plugins.  This keeps the track
visualisation independent of that optional plugin while preserving the exact
cone positions from the selected EUFS track model.
"""

from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray


class TrackMarkerPublisher(Node):
    def __init__(self):
        super().__init__('track_marker_publisher')
        default_path = Path(get_package_share_directory('eufs_tracks')) / 'models' / 'small_track' / 'model.sdf'
        self.declare_parameter('track_file', str(default_path))
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_rate', 1.0)
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(MarkerArray, '/track_markers', qos)
        self.markers = self._load_markers(
            Path(self.get_parameter('track_file').value),
            str(self.get_parameter('frame_id').value),
        )
        period = max(0.1, 1.0 / float(self.get_parameter('publish_rate').value))
        self.timer = self.create_timer(period, self._publish)
        self._publish()

    def _load_markers(self, path: Path, frame_id: str) -> MarkerArray:
        root = ET.parse(path).getroot()
        array = MarkerArray()
        for marker_id, include in enumerate(root.findall('.//include')):
            uri = include.findtext('uri', '')
            name = include.findtext('name', '')
            if 'cone' not in uri:
                continue
            pose = (include.findtext('pose', '0 0 0 0 0 0').split() + ['0'] * 6)[:6]
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.ns = 'eufs_track'
            marker.id = marker_id
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = float(pose[0])
            marker.pose.position.y = float(pose[1])
            marker.pose.position.z = 0.15
            marker.scale.x = 0.24
            marker.scale.y = 0.24
            marker.scale.z = 0.30
            marker.color.a = 1.0
            if 'yellow' in uri:
                marker.color.r, marker.color.g, marker.color.b = 1.0, 0.85, 0.0
            elif 'orange' in uri or 'big_cone' in uri:
                marker.color.r, marker.color.g, marker.color.b = 1.0, 0.35, 0.0
            elif 'blue' in uri:
                marker.color.r, marker.color.g, marker.color.b = 0.05, 0.25, 1.0
            else:
                marker.color.r, marker.color.g, marker.color.b = 0.7, 0.7, 0.7
            marker.text = name
            array.markers.append(marker)
        return array

    def _publish(self):
        stamp = self.get_clock().now().to_msg()
        for marker in self.markers.markers:
            marker.header.stamp = stamp
        self.publisher.publish(self.markers)


def main(args=None):
    rclpy.init(args=args)
    node = TrackMarkerPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
