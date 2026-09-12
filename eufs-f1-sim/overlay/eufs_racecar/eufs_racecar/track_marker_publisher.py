"""Publish the EUFS SDF cone layout using stock RViz MarkerArray messages.

The lean container does not include eufs_rviz_plugins.  This keeps the track
visualisation independent of that optional plugin while preserving the exact
cone positions from the selected EUFS track model.
"""

from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import yaml
from rclpy.node import Node
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Point
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
        boundaries = {'blue': [], 'yellow': []}
        # A track adapter opts into boundary markers by declaring the same
        # mesh metadata used by Gazebo.  This keeps stock tracks untouched and
        # lets future maps reuse the ordered-cone ribbon convention.
        marker_namespace = self._boundary_namespace(path)
        has_boundary_visual = marker_namespace is not None
        for marker_id, include in enumerate(root.findall('.//include')):
            uri = include.findtext('uri', '')
            name = include.findtext('name', '')
            if 'cone' not in uri:
                continue
            pose = (include.findtext('pose', '0 0 0 0 0 0').split() + ['0'] * 6)[:6]
            if has_boundary_visual and 'blue_cone' in uri:
                boundaries['blue'].append((float(pose[0]), float(pose[1])))
            elif has_boundary_visual and 'yellow_cone' in uri:
                boundaries['yellow'].append((float(pose[0]), float(pose[1])))
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.ns = 'eufs_track'
            marker.id = marker_id
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
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
        # Keep the existing cone markers intact and publish two closed strips
        # alongside them.  The SDF include order is the generator's increasing
        # s order, so each strip closes only its own finish seam and never
        # crosses the four-cone orange gate.
        for boundary_id, side in enumerate(('blue', 'yellow')):
            points = boundaries[side]
            if not points:
                continue
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.ns = marker_namespace
            marker.id = boundary_id
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.20
            marker.color.r = marker.color.g = marker.color.b = 1.0
            marker.color.a = 1.0
            for x, y in points + [points[0]]:
                point = Point()
                point.x = x
                point.y = y
                point.z = 0.008
                marker.points.append(point)
            array.markers.append(marker)
        return array

    @staticmethod
    def _boundary_namespace(model_path: Path):
        # Keep marker opt-in aligned with track_select/grid_geometry: a map
        # may attach metadata.yaml or provenance.yaml beside its model, or in
        # the sibling <track>/ attachment directory.
        roots = []
        if model_path.parent.parent.name == 'models':
            roots.append(model_path.parent.parent.parent / model_path.parent.name)
        roots.append(model_path.parent)
        provenance = {}
        for root in roots:
            for filename in ('metadata.yaml', 'provenance.yaml'):
                provenance_path = root / filename
                try:
                    with provenance_path.open(encoding='utf-8') as stream:
                        value = yaml.safe_load(stream) or {}
                except (OSError, ValueError, yaml.YAMLError):
                    continue
                if isinstance(value, dict):
                    provenance = value
                    break
            if provenance:
                break
        policy = provenance.get('cone_policy') or {}
        mesh_value = policy.get('white_strip_mesh') or provenance.get('boundary_strips')
        if not mesh_value:
            return None
        mesh_path = Path(str(mesh_value))
        if mesh_path.is_absolute():
            candidates = (mesh_path,)
        else:
            share = model_path.parent.parent.parent
            candidates = tuple(root / mesh_path for root in roots) + (
                model_path.parent / 'meshes' / mesh_path.name,
                share / mesh_path,
            )
        if not any(candidate.is_file() for candidate in candidates):
            return None
        namespace = policy.get('white_strip_marker_namespace')
        return str(namespace).strip() if namespace else 'eufs_track_boundary'

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
