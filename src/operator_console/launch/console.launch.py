"""Bring up the operator console."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='8080'),
        DeclareLaunchArgument('image_topic', default_value='/image_raw'),
        DeclareLaunchArgument('can_interface', default_value='can0'),
        Node(
            package='operator_console',
            executable='console_node',
            name='operator_console',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'image_topic': LaunchConfiguration('image_topic'),
                'can_interface': LaunchConfiguration('can_interface'),
            }],
        ),
    ])
