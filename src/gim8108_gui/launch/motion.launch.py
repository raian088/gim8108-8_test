from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gim8108_gui',
            executable='gim8108_motion_gui',
            name='gim8108_motion_gui',
            output='screen',
        ),
    ])
