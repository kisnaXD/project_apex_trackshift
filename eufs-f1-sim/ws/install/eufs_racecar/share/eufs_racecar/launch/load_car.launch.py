import os
from os.path import join

import yaml
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _arg(context, name):
    return LaunchConfiguration(name).perform(context)


def _forgez_defaults():
    cfg_path = join(get_package_share_directory('eufs_racecar'), 'config', 'forgez_battery.yaml')
    with open(cfg_path, 'r', encoding='utf-8') as stream:
        cfg = yaml.safe_load(stream)
    mode = cfg['forgez_battery']['default_mode']
    params = cfg['forgez_battery']['modes'][mode]
    return mode, params


def spawn_car(context, *args, **kwargs):
    namespace = _arg(context, 'namespace')
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
        },
    )
    robot_description = doc.toxml()

    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
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
                '-entity', namespace,
                '-topic', 'robot_description',
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
            name='joint_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description, 'rate': 200}],
            remappings=[('/joint_states', '/eufs/joint_states')],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='eufs'),
        DeclareLaunchArgument('launch_group', default_value='default'),
        DeclareLaunchArgument('robot_name', default_value='eufs'),
        DeclareLaunchArgument('vehicleModel', default_value='Ackermann'),
        DeclareLaunchArgument('commandMode', default_value='velocity'),
        DeclareLaunchArgument('vehicleModelConfig', default_value='configDry.yaml'),
        DeclareLaunchArgument('publish_gt_tf', default_value='false'),
        DeclareLaunchArgument('pub_ground_truth', default_value='true'),
        DeclareLaunchArgument('show_rqt_gui', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('forgez_mode', default_value='auto',
                              description='Forgez mode: Harvest, Nominal, Attack, or auto'),
        DeclareLaunchArgument('x', default_value='0'),
        DeclareLaunchArgument('y', default_value='0'),
        DeclareLaunchArgument('z', default_value='0.1'),
        DeclareLaunchArgument('roll', default_value='0'),
        DeclareLaunchArgument('pitch', default_value='0'),
        DeclareLaunchArgument('yaw', default_value='0'),
        Node(
            package='rviz2',
            executable='rviz2',
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
        OpaqueFunction(function=spawn_car),
    ])
