from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('can_channel',  default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('can_bustype',  default_value='slcan'),
        DeclareLaunchArgument('can_bitrate',  default_value='500000'),
        DeclareLaunchArgument('node_id',      default_value='0'),
        DeclareLaunchArgument('publish_rate', default_value='50.0'),

        Node(
            package='gim8108_driver',
            executable='gim8108_motor_node',
            name='gim8108_motor_node',
            parameters=[{
                'can_channel':  LaunchConfiguration('can_channel'),
                'can_bustype':  LaunchConfiguration('can_bustype'),
                'can_bitrate':  LaunchConfiguration('can_bitrate'),
                'node_id':      LaunchConfiguration('node_id'),
                'publish_rate': LaunchConfiguration('publish_rate'),
                'traj_accel': 5.0,
                'traj_decel': 5.0,
            }],
            output='screen',
        ),
    ])
