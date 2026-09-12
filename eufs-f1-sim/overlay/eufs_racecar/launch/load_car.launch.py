"""Single EUFS F1 launch: headless gzserver + dashboard; Start opens gzclient + RViz."""

from math import atan2, cos, degrees, hypot, sin
from os import environ
from os.path import isfile, join
import subprocess
from glob import glob
import csv
import xml.etree.ElementTree as ET

import yaml
import xacro

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from eufs_racecar.track_select import resolve_track
from eufs_racecar.grid_geometry import GridMapAdapter, build_grid_poses


def _arg(context, name):
    return LaunchConfiguration(name).perform(context)


def _namespace_path(namespace):
    clean = namespace.strip('/')
    return clean, f'/{clean}' if clean else ''


GRID_MAX_CARS = 20
GRID_ROW_SPACING_M = 12.0
GRID_COLUMN_STAGGER_M = 6.0
GRID_LATERAL_M = 2.5
BODY_FRONT_M = 4.4
BODY_REAR_M = 0.7
BODY_HALF_WIDTH_M = 1.05


def _read_geometry(path, fields):
    if not path or not isfile(path):
        return []
    with open(path, newline='', encoding='utf-8') as stream:
        return [tuple(float(row[field]) for field in fields)
                for row in csv.DictReader(stream)]


def _geometry_cycle(points):
    """Return the closed arclength of a sampled centreline/boundary."""
    if len(points) < 2:
        return 0.0
    return points[-1][0] + hypot(points[-1][1] - points[0][1],
                                 points[-1][2] - points[0][2])


def _arc_sample(points, s):
    """Interpolate a periodic (s, x, y, yaw) centreline sample."""
    cycle = _geometry_cycle(points)
    if cycle <= 0.0:
        raise ValueError('centreline has no usable arclength')
    s %= cycle
    for first, second in zip(points, points[1:]):
        if first[0] <= s <= second[0]:
            span = max(1.0e-9, second[0] - first[0])
            fraction = (s - first[0]) / span
            yaw_delta = atan2(sin(second[3] - first[3]), cos(second[3] - first[3]))
            return (
                first[1] + fraction * (second[1] - first[1]),
                first[2] + fraction * (second[2] - first[2]),
                first[3] + fraction * yaw_delta,
            )
    first = points[-1]
    second = (cycle, points[0][1], points[0][2], points[0][3])
    fraction = (s - first[0]) / max(1.0e-9, second[0] - first[0])
    yaw_delta = atan2(sin(second[3] - first[3]), cos(second[3] - first[3]))
    return (
        first[1] + fraction * (second[1] - first[1]),
        first[2] + fraction * (second[2] - first[2]),
        first[3] + fraction * yaw_delta,
    )


def _rigid_transform(point, gate_x, gate_y, yaw_delta):
    px, py = point
    c, s = cos(yaw_delta), sin(yaw_delta)
    return gate_x + c * px - s * py, gate_y + s * px + c * py


def _nearest_arclength(points, x, y, gate_x, gate_y, yaw_delta):
    """Project a spawn point onto the transformed periodic centreline."""
    cycle = _geometry_cycle(points)
    best_distance = float('inf')
    best_s = 0.0
    segments = list(zip(points, points[1:]))
    segments.append((points[-1], (cycle, points[0][1], points[0][2], points[0][3])))
    for first, second in segments:
        p1 = _rigid_transform(first[1:3], gate_x, gate_y, yaw_delta)
        p2 = _rigid_transform(second[1:3], gate_x, gate_y, yaw_delta)
        dx, dy = p2[0] - p1[0], p2[1] - p1[1]
        scale = max(1.0e-9, dx * dx + dy * dy)
        fraction = max(0.0, min(1.0, ((x - p1[0]) * dx + (y - p1[1]) * dy) / scale))
        px, py = p1[0] + fraction * dx, p1[1] + fraction * dy
        distance = hypot(x - px, y - py)
        if distance < best_distance:
            best_distance = distance
            best_s = first[0] + fraction * (second[0] - first[0])
    return best_s


def _grid_pose(index, x, y, yaw, num_cars=1, centerline=None, map_adapter=None):
    """Place a car in the two-column, staggered grid.

    A one-car launch intentionally retains the historical spawn exactly.  A
    multi-car launch offsets the ego to the first pole and follows the closed
    track centreline backwards, so the last row remains on a curved map
    instead of assuming the start tangent stays straight.
    """
    if num_cars == 1:
        return x, y, yaw
    column = index % 2
    row = index // 2
    side = -1.0 if column == 0 else 1.0
    longitudinal = GRID_ROW_SPACING_M * row + GRID_COLUMN_STAGGER_M * column
    if centerline:
        # Attached centreline and boundary CSVs use world/map XY with
        # the S/F gate at (0, 0).  Do not rigidly transform this geometry when
        # a caller overrides x/y/yaw; only N=1 is required to honour a custom
        # pose exactly.
        cycle = _geometry_cycle(centerline)
        nominal_s = map_adapter.nominal_spawn_s() if map_adapter else None
        if nominal_s is None:
            nominal_s = (map_adapter.spawn_arclength(x, y) if map_adapter
                         else _nearest_arclength(centerline, x, y, 0.0, 0.0, 0.0))
        nominal_x, nominal_y, nominal_yaw = _arc_sample(centerline, nominal_s)
        # resolve_track places an authored gate spawn five metres before
        # the gate.  Preserve that authored arclength (the sampled point can
        # differ by a few tenths due to the closing segment); custom poses are
        # projected onto the raw map centreline instead.
        if (hypot(x - nominal_x, y - nominal_y) <= 1.0 and
                abs(atan2(sin(yaw - nominal_yaw), cos(yaw - nominal_yaw))) <= 0.1):
            spawn_s = nominal_s
        else:
            spawn_s = (map_adapter.spawn_arclength(x, y)
                       if map_adapter else _nearest_arclength(centerline, x, y, 0.0, 0.0, 0.0))
        cx, cy, tangent = _arc_sample(centerline, spawn_s - longitudinal)
        px = cx + side * GRID_LATERAL_M * -sin(tangent)
        py = cy + side * GRID_LATERAL_M * cos(tangent)
        return px, py, tangent
    forward = (cos(yaw), sin(yaw))
    left = (-sin(yaw), cos(yaw))
    return (x - longitudinal * forward[0] + side * GRID_LATERAL_M * left[0],
            y - longitudinal * forward[1] + side * GRID_LATERAL_M * left[1], yaw)


def _point_in_loop(x, y, loop):
    inside = False
    for (x1, y1), (x2, y2) in zip(loop, loop[1:] + loop[:1]):
        if ((y1 > y) != (y2 > y)) and x < (x2 - x1) * (y - y1) / (y2 - y1) + x1:
            inside = not inside
    return inside


def _inside_track(x, y, left_loop, right_loop):
    # The two boundary loops are intentionally tested independently.  Joining
    # their endpoints creates a long artificial polygon across the finish.
    return _point_in_loop(x, y, left_loop) ^ _point_in_loop(x, y, right_loop)


def _rect_corners(pose):
    x, y, yaw = pose
    c, s = cos(yaw), sin(yaw)
    return [
        (x + longitudinal * c - lateral * s,
         y + longitudinal * s + lateral * c)
        for longitudinal in (-BODY_REAR_M, BODY_FRONT_M)
        for lateral in (-BODY_HALF_WIDTH_M, BODY_HALF_WIDTH_M)
    ]


def _rectangles_overlap(first, second):
    """Strict oriented-rectangle intersection test for grid validation."""
    corners_a, corners_b = _rect_corners(first), _rect_corners(second)
    axes = []
    for _, _, angle in (first, second):
        axes.extend(((cos(angle), sin(angle)), (-sin(angle), cos(angle))))
    for ax, ay in axes:
        a = [px * ax + py * ay for px, py in corners_a]
        b = [px * ax + py * ay for px, py in corners_b]
        if max(a) <= min(b) or max(b) <= min(a):
            return False
    return True


def _validate_grid(poses, centerline, boundaries, gate_x, gate_y, yaw_delta):
    if not centerline or not boundaries:
        return
    left_raw = _read_geometry(boundaries, ('s_m', 'left_x_m', 'left_y_m'))
    right_raw = _read_geometry(boundaries, ('s_m', 'right_x_m', 'right_y_m'))
    if len(left_raw) < 3 or len(right_raw) < 3:
        raise RuntimeError('boundary metadata is too short for grid validation')
    left_loop = [_rigid_transform(row[1:], gate_x, gate_y, yaw_delta) for row in left_raw]
    right_loop = [_rigid_transform(row[1:], gate_x, gate_y, yaw_delta) for row in right_raw]
    for index, pose in enumerate(poses):
        if not all(_inside_track(px, py, left_loop, right_loop)
                   for px, py in _rect_corners(pose)):
            raise RuntimeError(
                f'grid car {index} body ({BODY_FRONT_M}m front/{BODY_REAR_M}m rear/'
                f'{BODY_HALF_WIDTH_M}m half-width) leaves track boundaries'
            )
    for first in range(len(poses)):
        for second in range(first + 1, len(poses)):
            if _rectangles_overlap(poses[first], poses[second]):
                raise RuntimeError(f'grid cars {first} and {second} overlap')


def _forgez_defaults():
    cfg_path = join(get_package_share_directory('eufs_racecar'), 'config', 'forgez_battery.yaml')
    with open(cfg_path, 'r', encoding='utf-8') as stream:
        cfg = yaml.safe_load(stream)
    mode = cfg['forgez_battery']['default_mode']
    params = cfg['forgez_battery']['modes'][mode]
    return mode, params


def _prepend_env(name, *paths):
    ordered = []
    for item in list(paths) + environ.get(name, '').split(':'):
        if item and item not in ordered:
            ordered.append(item)
    environ[name] = ':'.join(ordered)


def _share_or_empty(package_name):
    try:
        return get_package_share_directory(package_name)
    except Exception:
        return ''


def _lib_or_empty(package_name):
    try:
        return join(get_package_prefix(package_name), 'lib')
    except Exception:
        return ''


_SILVER = {
    'script': 'EUFSF1/Silver',
    'ambient': '0.75 0.75 0.78 1',
    'diffuse': '0.75 0.75 0.78 1',
    'specular': '0.90 0.90 0.95 1',
}
_TIRE_BLACK = {
    'script': 'EUFSF1/TireBlack',
    'ambient': '0.05 0.05 0.05 1',
    'diffuse': '0.05 0.05 0.05 1',
    'specular': '0.08 0.08 0.08 1',
}
_LINK_PAINT = {
    # gz sdf -p lumps chassis + wings onto base_link.
    'base_link': _SILVER,
    'chassis': _SILVER,
    'front_wing': _SILVER,
    'rear_wing': _SILVER,
    'left_rear_wheel': _TIRE_BLACK,
    'right_rear_wheel': _TIRE_BLACK,
    'left_front_wheel': _TIRE_BLACK,
    'right_front_wheel': _TIRE_BLACK,
}
_GZ_MATERIAL_URI = 'file://media/materials/scripts/gazebo.material'


def _set_visual_paint(material_el, paint):
    """Classic script (gazebo.material) plus RGBA so gzclient still paints if OGRE misses the name."""
    material_el.clear()
    script = ET.SubElement(material_el, 'script')
    ET.SubElement(script, 'uri').text = _GZ_MATERIAL_URI
    ET.SubElement(script, 'name').text = paint['script']
    ET.SubElement(material_el, 'ambient').text = paint['ambient']
    ET.SubElement(material_el, 'diffuse').text = paint['diffuse']
    ET.SubElement(material_el, 'specular').text = paint['specular']
    ET.SubElement(material_el, 'emissive').text = '0 0 0 1'
    ET.SubElement(material_el, 'lighting').text = '1'


def _sdf_with_rviz_paint(urdf_path, sdf_path):
    """Force silver/black on named links. URDF→SDF otherwise leaves STLs Gazebo-white."""
    converted = subprocess.run(
        ['gz', 'sdf', '-p', urdf_path],
        check=True, capture_output=True, text=True,
    )
    racecar_meshes = join(get_package_share_directory('eufs_racecar'), 'meshes')
    root = ET.fromstring(converted.stdout)
    for uri_el in root.iter('uri'):
        text = uri_el.text or ''
        if 'eufs_racecar/meshes/' in text:
            uri_el.text = f'file://{racecar_meshes}/{text.rsplit("/", 1)[-1]}'
    for link in root.iter('link'):
        paint = _LINK_PAINT.get(link.get('name'))
        if paint is None:
            continue
        visuals = list(link.findall('visual'))
        if not visuals:
            visuals = [ET.SubElement(link, 'visual')]
        for visual in visuals:
            material_el = visual.find('material')
            if material_el is None:
                material_el = ET.SubElement(visual, 'material')
            _set_visual_paint(material_el, paint)
    ET.indent(root, space='  ')
    with open(sdf_path, 'w', encoding='utf-8') as stream:
        stream.write("<?xml version='1.0'?>\n")
        stream.write(ET.tostring(root, encoding='unicode'))
        stream.write('\n')
    return sdf_path


def _prepare_gazebo_env():
    racecar = _share_or_empty('eufs_racecar')
    tracks = _share_or_empty('eufs_tracks')
    sensors = _share_or_empty('eufs_sensors')
    battery_lib = _lib_or_empty('gazebo_ros_battery')
    plugins_lib = _lib_or_empty('eufs_plugins')
    humble_lib = '/opt/ros/humble/lib'

    _prepend_env('GAZEBO_PLUGIN_PATH', battery_lib, plugins_lib, humble_lib)
    _prepend_env('GAZEBO_MODEL_PATH', join(tracks, 'models') if tracks else '', tracks, racecar)
    _prepend_env(
        'GAZEBO_MATERIAL_PATH',
        join(racecar, 'materials', 'scripts') if racecar else '',
        '/usr/share/gazebo-11/media/materials/scripts',
    )
    _prepend_env(
        'GAZEBO_RESOURCE_PATH',
        join(sensors, 'meshes') if sensors else '',
        join(tracks, 'meshes') if tracks else '',
        join(tracks, 'materials') if tracks else '',
        tracks,
        join(racecar, 'meshes') if racecar else '',
        join(racecar, 'materials') if racecar else '',
        racecar,
        '/usr/share/gazebo-11',
    )
    # Default models.gazebosim.org makes gzclient sit on "Preparing your world".
    environ['GAZEBO_MODEL_DATABASE_URI'] = ''
    environ['LIBGL_DRI3_DISABLE'] = '1'


def _spawn_nodes(
    namespace, entity, x, y, z, roll, pitch, yaw, forgez_mode, publish_tf,
    vehicle_model, command_mode, config_file, noise_config, pub_ground_truth, frame_prefix,
    native_remap,
):
    namespace_clean, _ns_path = _namespace_path(namespace)
    entity = entity or namespace_clean or 'eufs'

    if forgez_mode == 'auto':
        forgez_mode, forgez_params = _forgez_defaults()
    else:
        cfg_path = join(get_package_share_directory('eufs_racecar'), 'config', 'forgez_battery.yaml')
        with open(cfg_path, 'r', encoding='utf-8') as stream:
            cfg = yaml.safe_load(stream)
        forgez_params = cfg['forgez_battery']['modes'][forgez_mode]

    xacro_path = join(get_package_share_directory('eufs_racecar'), 'robots', 'eufs', 'robot.urdf.xacro')
    doc = xacro.process_file(
        xacro_path,
        mappings={
            'forgez_mode': forgez_mode,
            'forgez_T_core': str(forgez_params['T_core']),
            'forgez_E_lap': str(forgez_params['E_lap']),
            'forgez_R_OT': str(forgez_params['R_OT']),
            'forgez_max_power_w': str(forgez_params['max_power_w']),
            'robot_namespace': namespace_clean,
            'vehicle_model': vehicle_model,
            'command_mode': command_mode,
            'use_ackermann': str(vehicle_model == 'Ackermann').lower(),
            'config_file': config_file,
            'noise_config': noise_config,
            # Gazebo's plugin must never publish odom TF; the adapter below
            # owns map -> odom -> base_link.  Keep this mapping false even
            # when the adapter is enabled.
            'publish_tf': 'false',
            'frame_prefix': frame_prefix,
            'pub_ground_truth': str(pub_ground_truth).lower(),
            'native_remap': str(native_remap).lower(),
        },
    )
    robot_description = doc.toxml()
    racecar_meshes = join(get_package_share_directory('eufs_racecar'), 'meshes')
    gazebo_description = robot_description.replace(
        'package://eufs_racecar/meshes/',
        f'file://{racecar_meshes}/',
    )
    stem = namespace_clean or 'root'
    urdf_path = f'/tmp/eufs_robot_description_{stem}.urdf'
    sdf_path = f'/tmp/eufs_robot_description_{stem}.sdf'
    with open(urdf_path, 'w', encoding='utf-8') as stream:
        stream.write(gazebo_description)
    _sdf_with_rviz_paint(urdf_path, sdf_path)

    nodes = [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=namespace_clean,
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'robot_description': robot_description,
                'frame_prefix': frame_prefix,
            }],
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name=f'spawn_robot_{stem}',
            output='screen',
            arguments=[
                '-entity', entity,
                '-file', sdf_path,
                '-x', str(x),
                '-y', str(y),
                '-z', str(z),
                '-R', str(roll),
                '-P', str(pitch),
                '-Y', str(yaw),
                '-timeout', '180.0',
                '--ros-args', '--log-level', 'warn',
            ],
        ),
    ]
    if publish_tf:
        nodes.append(
            Node(
                package='eufs_racecar',
                executable='odom_tf_publisher',
                name=f'odom_tf_{stem}',
                output='screen',
                parameters=[{
                    'use_sim_time': False,
                    'namespace': namespace_clean,
                    'x': float(x),
                    'y': float(y),
                    'z': float(z),
                    'yaw': float(yaw),
                    'odom_relative': vehicle_model == 'DynamicBicycle',
                    'frame_prefix': frame_prefix,
                }],
            ),
        )
    return nodes


def _launch_stack(context, *args, **kwargs):
    track = _arg(context, 'track')
    num_cars = int(_arg(context, 'num_cars') or _arg(context, 'cars') or '1')
    if not 1 <= num_cars <= GRID_MAX_CARS:
        raise RuntimeError(f'num_cars must be between 1 and {GRID_MAX_CARS}')
    assets = resolve_track(track)

    world = _arg(context, 'world') or assets['world']
    track_file = _arg(context, 'track_file') or assets['track_file']
    x = float(_arg(context, 'x') or assets['x'])
    y = float(_arg(context, 'y') or assets['y'])
    yaw = float(_arg(context, 'yaw') or assets['yaw'])
    z = _arg(context, 'z') or '0.08'
    roll = _arg(context, 'roll')
    pitch = _arg(context, 'pitch')
    forgez_mode = _arg(context, 'forgez_mode')
    vehicle_model = _arg(context, 'vehicleModel')
    command_mode = _arg(context, 'commandMode')
    if vehicle_model not in ('Ackermann', 'DynamicBicycle'):
        raise RuntimeError(
            f"vehicleModel must be Ackermann or DynamicBicycle, got {vehicle_model!r}"
        )
    if command_mode not in ('acceleration', 'velocity'):
        raise RuntimeError(
            f"commandMode must be acceleration or velocity, got {command_mode!r}"
        )
    config_name = _arg(context, 'vehicleModelConfig') or (
        'f1_dynamic_bicycle.yaml' if vehicle_model == 'DynamicBicycle' else 'configDry.yaml'
    )
    config_file = config_name if config_name.startswith('/') else join(
        get_package_share_directory('eufs_racecar'), 'robots', 'eufs', config_name
    )
    models_share = _share_or_empty('eufs_models')
    noise_config = join(models_share, 'config', 'noise.yaml') if models_share else ''
    pub_ground_truth = _arg(context, 'pub_ground_truth').lower() == 'true'
    if vehicle_model == 'DynamicBicycle':
        plugin = 'libgazebo_race_car_model.so'
        paths = [p for p in environ.get('GAZEBO_PLUGIN_PATH', '').split(':') if p]
        if not any(glob(join(path, plugin)) for path in paths):
            raise RuntimeError(f'Requested DynamicBicycle plugin is unavailable: {plugin}')
    if forgez_mode == 'auto':
        forgez_mode_name, forgez_params = _forgez_defaults()
    else:
        forgez_mode_name = forgez_mode
        cfg_path = join(get_package_share_directory('eufs_racecar'), 'config', 'forgez_battery.yaml')
        with open(cfg_path, 'r', encoding='utf-8') as stream:
            forgez_params = yaml.safe_load(stream)['forgez_battery']['modes'][forgez_mode]
    base_ns = _arg(context, 'namespace') or 'eufs'
    robot_name = _arg(context, 'robot_name')
    centerline = _read_geometry(
        assets.get('centerline', ''), ('s_m', 'x_m', 'y_m', 'yaw_rad'),
    )
    map_adapter = GridMapAdapter.from_assets(assets)
    boundaries = assets.get('boundaries', '')
    grid_poses = build_grid_poses(assets, num_cars, spawn=(x, y, yaw))
    if num_cars > 1 and centerline and boundaries:
        _validate_grid(
            grid_poses, centerline, boundaries, 0.0, 0.0, 0.0,
        )

    rviz_config_file = join(
        get_package_share_directory('eufs_racecar'), 'config', 'eufs_f1.rviz',
    )
    gz_launch_dir = join(get_package_share_directory('gazebo_ros'), 'launch')

    # Headless gzserver only. gzclient and RViz open from start_dashboard Start.
    # Do not Include gzclient.launch / eufs_tracks/*.launch / eufs_launcher.
    actions = [
        LogInfo(msg=[
            f'Spawn {track} x={x:.6f} y={y:.6f} z={z} yaw={yaw:.6f} rad '
            f'({degrees(yaw):.2f} deg) heading along {track} S/F'
        ]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(join(gz_launch_dir, 'gzserver.launch.py')),
            launch_arguments={
                'world': world,
                'verbose': 'false',
                'pause': 'true',
            }.items(),
        ),
        Node(
            package='eufs_racecar',
            executable='track_marker_publisher',
            name='track_marker_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'track_file': track_file,
                'frame_id': 'map',
                'publish_rate': 1.0,
            }],
        ),
        Node(
            package='eufs_racecar',
            executable='start_dashboard',
            name='start_dashboard',
            condition=IfCondition(LaunchConfiguration('show_dashboard')),
            output='screen',
            additional_env={
                'DISPLAY': environ.get('DISPLAY', ':0'),
                'QT_X11_NO_MITSHM': environ.get('QT_X11_NO_MITSHM', '1'),
                'LIBGL_DRI3_DISABLE': environ.get('LIBGL_DRI3_DISABLE', '1'),
            },
            parameters=[{
                'use_sim_time': True,
                'track': track,
                # String: Humble params YAML cannot override an INTEGER default.
                'cars': str(num_cars),
                'namespace': base_ns,
                'vehicle_model': vehicle_model,
                'rviz_config': rviz_config_file,
                'forgez_mode': forgez_mode_name,
                'forgez_T_core': str(forgez_params['T_core']),
                'forgez_E_lap': str(forgez_params['E_lap']),
                'forgez_R_OT': str(forgez_params['R_OT']),
            }],
        ),
    ]
    for index in range(num_cars):
        namespace = base_ns if index == 0 else f'{base_ns}{index + 1}'
        entity = (robot_name or namespace) if index == 0 else f'{robot_name or base_ns}{index + 1}'
        spawn_x, spawn_y, spawn_yaw = grid_poses[index]
        actions.extend(_spawn_nodes(
                namespace=namespace,
                entity=entity,
                x=spawn_x,
                y=spawn_y,
                z=z,
                roll=roll,
                pitch=pitch,
                yaw=spawn_yaw,
                forgez_mode=forgez_mode,
                publish_tf=True,
                vehicle_model=vehicle_model,
                command_mode=command_mode,
                config_file=config_file,
                noise_config=noise_config,
                pub_ground_truth=pub_ground_truth,
                frame_prefix='' if index == 0 else f'{namespace}/',
                native_remap=index != 0,
            ))
        actions.extend([
            Node(
                package='eufs_racecar',
                executable='ackermann_cmd_bridge',
                name=f'ackermann_cmd_bridge_{namespace}',
                namespace=namespace,
                output='screen',
                parameters=[{
                    'use_sim_time': False,
                    'namespace': namespace,
                }],
            ),
            Node(
                package='eufs_racecar',
                executable='tyre_state_publisher',
                name=f'tyre_state_publisher_{namespace}',
                namespace=namespace,
                output='screen',
                parameters=[{
                    'use_sim_time': False,
                    'namespace': namespace,
                    'lap_length_m': 5513.0,
                }],
            ),
        ])
    return actions


def generate_launch_description():
    _prepare_gazebo_env()
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='eufs'),
        DeclareLaunchArgument('launch_group', default_value='default'),
        DeclareLaunchArgument('robot_name', default_value='eufs'),
        DeclareLaunchArgument('vehicleModel', default_value='DynamicBicycle'),
        DeclareLaunchArgument('commandMode', default_value='acceleration'),
        DeclareLaunchArgument(
            'vehicleModelConfig', default_value='',
            description='Vehicle YAML; defaults to the F1 DynamicBicycle config only in that mode.',
        ),
        DeclareLaunchArgument('publish_gt_tf', default_value='false'),
        DeclareLaunchArgument('pub_ground_truth', default_value='true'),
        DeclareLaunchArgument(
            'show_dashboard', default_value='true',
            description='Start the managed dashboard node when true.',
        ),
        DeclareLaunchArgument(
            'show_rqt_gui',
            default_value='false',
            description='Ignored: rqt is not auto-started. Start opens gzclient + RViz.',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='Ignored: RViz starts from the dashboard Start button.',
        ),
        DeclareLaunchArgument(
            'gazebo_gui',
            default_value='false',
            description='Ignored: gzclient starts from the dashboard Start button.',
        ),
        DeclareLaunchArgument(
            'track',
            default_value='cota',
            description='Track name discovered from eufs_tracks world/model/CSV assets.',
        ),
        DeclareLaunchArgument('num_cars', default_value=''),
        DeclareLaunchArgument('cars', default_value='1'),
        DeclareLaunchArgument('world', default_value=''),
        DeclareLaunchArgument('track_file', default_value=''),
        DeclareLaunchArgument('forgez_mode', default_value='auto'),
        DeclareLaunchArgument('x', default_value=''),
        DeclareLaunchArgument('y', default_value=''),
        DeclareLaunchArgument('z', default_value='0.08'),
        DeclareLaunchArgument('roll', default_value='0'),
        DeclareLaunchArgument('pitch', default_value='0'),
        DeclareLaunchArgument('yaw', default_value=''),
        OpaqueFunction(function=_launch_stack),
    ])
