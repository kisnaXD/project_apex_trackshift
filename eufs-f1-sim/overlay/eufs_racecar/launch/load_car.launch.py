"""Single EUFS F1 launch: Gazebo + RViz + one robot_description + one spawn."""

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
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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


def _sdf_with_rviz_paint(urdf_path):
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
    sdf_path = '/tmp/eufs_robot_description.sdf'
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


def spawn_car(context, *args, **kwargs):
    namespace = _arg(context, 'namespace')
    namespace_clean, namespace_path = _namespace_path(namespace)
    entity = _arg(context, 'robot_name') or namespace_clean or 'eufs'
    x = _arg(context, 'x')
    y = _arg(context, 'y')
    z = _arg(context, 'z')
    roll = _arg(context, 'roll')
    pitch = _arg(context, 'pitch')
    yaw = _arg(context, 'yaw')
    forgez_mode = _arg(context, 'forgez_mode')
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
    urdf_path = '/tmp/eufs_robot_description.urdf'
    with open(urdf_path, 'w', encoding='utf-8') as stream:
        stream.write(gazebo_description)
    sdf_path = _sdf_with_rviz_paint(urdf_path)
    joint_states_topic = f'{namespace_path}/joint_states' if namespace_path else '/joint_states'

    return [
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
            name='spawn_robot',
            output='screen',
            arguments=[
                '-entity', entity,
                '-file', sdf_path,
                '-x', x,
                '-y', y,
                '-z', z,
                '-R', roll,
                '-P', pitch,
                '-Y', yaw,
                '-spawn_service_timeout', '60.0',
                '--ros-args', '--log-level', 'warn',
            ],
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            namespace=namespace_clean,
            name='joint_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'robot_description': robot_description,
                'rate': 200,
            }],
            remappings=[('/joint_states', joint_states_topic)],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_publisher',
            output='screen',
            arguments=['0.0', '0.0', '0.0', '0.0', '0', '0', 'map', 'odom'],
        ),
    ]


def generate_launch_description():
    _prepare_gazebo_env()

    rqt_perspective_file = join(
        get_package_share_directory('eufs_rqt'), 'config', 'eufs_sim.perspective',
    )
    rviz_config_file = join(
        get_package_share_directory('eufs_racecar'), 'config', 'eufs_f1.rviz',
    )
    default_world = join(
        get_package_share_directory('eufs_tracks'), 'worlds', 'small_track.world',
    )
    gz_launch_dir = join(get_package_share_directory('gazebo_ros'), 'launch')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='eufs'),
        DeclareLaunchArgument('launch_group', default_value='default'),
        DeclareLaunchArgument('robot_name', default_value='eufs'),
        DeclareLaunchArgument('vehicleModel', default_value='Ackermann'),
        DeclareLaunchArgument('commandMode', default_value='velocity'),
        DeclareLaunchArgument('vehicleModelConfig', default_value='configDry.yaml'),
        DeclareLaunchArgument('publish_gt_tf', default_value='false'),
        DeclareLaunchArgument('pub_ground_truth', default_value='true'),
        DeclareLaunchArgument('show_rqt_gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('gazebo_gui', default_value='true'),
        DeclareLaunchArgument('world', default_value=default_world),
        DeclareLaunchArgument(
            'track_file',
            default_value=join(
                get_package_share_directory('eufs_tracks'),
                'models', 'small_track', 'model.sdf'),
        ),
        DeclareLaunchArgument('forgez_mode', default_value='auto'),
        DeclareLaunchArgument('x', default_value='-13.0'),
        DeclareLaunchArgument('y', default_value='10.3'),
        DeclareLaunchArgument('z', default_value='0.1'),
        DeclareLaunchArgument('roll', default_value='0'),
        DeclareLaunchArgument('pitch', default_value='0'),
        DeclareLaunchArgument('yaw', default_value='0'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(join(gz_launch_dir, 'gzserver.launch.py')),
            launch_arguments={
                'world': LaunchConfiguration('world'),
                'verbose': 'false',
                'pause': 'false',
            }.items(),
        ),
        TimerAction(
            period=2.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(join(gz_launch_dir, 'gzclient.launch.py')),
                    condition=IfCondition(LaunchConfiguration('gazebo_gui')),
                    launch_arguments={'verbose': 'false'}.items(),
                ),
            ],
        ),
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz',
                    arguments=['-d', rviz_config_file],
                    parameters=[{'use_sim_time': True}],
                    condition=IfCondition(LaunchConfiguration('rviz')),
                ),
                Node(
                    package='rqt_gui',
                    executable='rqt_gui',
                    name='eufs_sim_rqt',
                    output='screen',
                    arguments=['--force-discover', '--perspective-file', rqt_perspective_file],
                    condition=IfCondition(LaunchConfiguration('show_rqt_gui')),
                ),
            ],
        ),
        Node(
            package='eufs_racecar',
            executable='track_marker_publisher',
            name='track_marker_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'track_file': LaunchConfiguration('track_file'),
                'frame_id': 'map',
                'publish_rate': 1.0,
            }],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
        Node(
            package='eufs_racecar',
            executable='start_dashboard',
            name='start_dashboard',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'track': LaunchConfiguration('track', default='cota'),
                'cars': LaunchConfiguration('cars', default='1'),
                'namespace': LaunchConfiguration('namespace'),
            }],
        ),
        OpaqueFunction(function=spawn_car),
    ])
