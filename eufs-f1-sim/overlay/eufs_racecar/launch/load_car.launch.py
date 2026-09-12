"""Single EUFS F1 launch: headless gzserver + dashboard; Start opens gzclient + RViz."""

from math import cos, degrees, sin
from os import environ
from os.path import join
import subprocess
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
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from eufs_racecar.track_select import resolve_track


def _arg(context, name):
    return LaunchConfiguration(name).perform(context)


def _namespace_path(namespace):
    clean = namespace.strip('/')
    return clean, f'/{clean}' if clean else ''


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


def _spawn_nodes(namespace, entity, x, y, z, roll, pitch, yaw, forgez_mode, publish_tf):
    namespace_clean, _ns_path = _namespace_path(namespace)
    entity = entity or namespace_clean or 'eufs'
    config_file = join(get_package_share_directory('eufs_racecar'), 'robots', 'eufs', 'configDry.yaml')

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
            'config_file': config_file,
            'forgez_mode': forgez_mode,
            'forgez_T_core': str(forgez_params['T_core']),
            'forgez_E_lap': str(forgez_params['E_lap']),
            'forgez_R_OT': str(forgez_params['R_OT']),
            'forgez_max_power_w': str(forgez_params['max_power_w']),
            'robot_namespace': namespace_clean,
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
                package='tf2_ros',
                executable='static_transform_publisher',
                name='map_to_odom_publisher',
                output='screen',
                parameters=[{'use_sim_time': True}],
                arguments=['0.0', '0.0', '0.0', '0.0', '0', '0', 'map', 'odom'],
            ),
        )
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
                }],
            ),
        )
    return nodes


def _launch_stack(context, *args, **kwargs):
    track = _arg(context, 'track')
    num_cars = int(_arg(context, 'num_cars') or _arg(context, 'cars') or '1')
    if num_cars < 1:
        raise RuntimeError('num_cars must be >= 1')
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
    if forgez_mode == 'auto':
        forgez_mode_name, forgez_params = _forgez_defaults()
    else:
        forgez_mode_name = forgez_mode
        cfg_path = join(get_package_share_directory('eufs_racecar'), 'config', 'forgez_battery.yaml')
        with open(cfg_path, 'r', encoding='utf-8') as stream:
            forgez_params = yaml.safe_load(stream)['forgez_battery']['modes'][forgez_mode]
    base_ns = _arg(context, 'namespace') or 'eufs'
    robot_name = _arg(context, 'robot_name')
    left = (-sin(yaw), cos(yaw))

    rviz_config_file = join(
        get_package_share_directory('eufs_racecar'), 'config', 'eufs_f1.rviz',
    )
    gz_launch_dir = join(get_package_share_directory('gazebo_ros'), 'launch')

    # Headless gzserver only. gzclient and RViz open from start_dashboard Start.
    # Do not Include gzclient.launch / eufs_tracks/*.launch / eufs_launcher.
    actions = [
        LogInfo(msg=[
            f'Spawn {track} x={x:.6f} y={y:.6f} z={z} yaw={yaw:.6f} rad '
            f'({degrees(yaw):.2f} deg) heading along COTA S/F'
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
                'rviz_config': rviz_config_file,
                'forgez_mode': forgez_mode_name,
                'forgez_T_core': str(forgez_params['T_core']),
                'forgez_E_lap': str(forgez_params['E_lap']),
                'forgez_R_OT': str(forgez_params['R_OT']),
            }],
        ),
        Node(
            package='eufs_racecar',
            executable='ackermann_cmd_bridge',
            name='ackermann_cmd_bridge',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'namespace': base_ns,
            }],
        ),
        Node(
            package='eufs_racecar',
            executable='tyre_state_publisher',
            name='tyre_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'namespace': base_ns,
                'lap_length_m': 5513.0,
            }],
        ),
    ]
    for index in range(num_cars):
        namespace = base_ns if index == 0 else f'{base_ns}{index + 1}'
        entity = (robot_name or namespace) if index == 0 else f'{robot_name or base_ns}{index + 1}'
        actions.extend(
            _spawn_nodes(
                namespace=namespace,
                entity=entity,
                x=x + left[0] * 3.0 * index,
                y=y + left[1] * 3.0 * index,
                z=z,
                roll=roll,
                pitch=pitch,
                yaw=yaw,
                forgez_mode=forgez_mode,
                publish_tf=(index == 0),
            )
        )
    return actions


def generate_launch_description():
    _prepare_gazebo_env()
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='eufs'),
        DeclareLaunchArgument('launch_group', default_value='default'),
        DeclareLaunchArgument('robot_name', default_value='eufs'),
        DeclareLaunchArgument('vehicleModel', default_value='Ackermann'),
        DeclareLaunchArgument('commandMode', default_value='velocity'),
        DeclareLaunchArgument('vehicleModelConfig', default_value='configDry.yaml'),
        DeclareLaunchArgument('publish_gt_tf', default_value='false'),
        DeclareLaunchArgument('pub_ground_truth', default_value='true'),
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
            description='cota or small_track. Selected on this launch only; no eufs_tracks/*.launch.',
        ),
        DeclareLaunchArgument('num_cars', default_value='1'),
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
