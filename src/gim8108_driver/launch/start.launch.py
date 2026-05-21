"""
GIM8108-8 一発起動ランチファイル
Usage:
  ros2 launch gim8108_driver start.launch.py
  ros2 launch gim8108_driver start.launch.py gear_ratio:=8.0 traj_vel:=20.0
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('axis',            default_value='0'),
        DeclareLaunchArgument('publish_rate',    default_value='50.0'),
        DeclareLaunchArgument('connect_timeout', default_value='60.0'),
        DeclareLaunchArgument('traj_vel',        default_value='10.0'),
        DeclareLaunchArgument('traj_accel',      default_value='5.0'),
        DeclareLaunchArgument('traj_decel',      default_value='5.0'),
        DeclareLaunchArgument('gear_ratio',      default_value='8.0'),

        Node(
            package='gim8108_driver',
            executable='gim8108_motor_node_usb',
            name='gim8108_motor_node',
            parameters=[{
                'axis':            LaunchConfiguration('axis'),
                'publish_rate':    LaunchConfiguration('publish_rate'),
                'connect_timeout': LaunchConfiguration('connect_timeout'),
                'traj_vel':        LaunchConfiguration('traj_vel'),
                'traj_accel':      LaunchConfiguration('traj_accel'),
                'traj_decel':      LaunchConfiguration('traj_decel'),
                'gear_ratio':      LaunchConfiguration('gear_ratio'),
            }],
            output='screen',
        ),

        Node(
            package='gim8108_gui',
            executable='gim8108_control_gui',
            name='gim8108_gui',
            output='screen',
        ),
    ])
