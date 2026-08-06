"""Bring up the Argus camera node with the default parameter file."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('jetson_camera'), 'config', 'camera_params.yaml'
    )

    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Parameter file for the Argus camera node',
        ),
        Node(
            package='jetson_camera',
            executable='argus_camera_node',
            name='argus_camera',
            parameters=[params_file],
            output='screen',
            # The node exits rather than lingering as a zombie when capture is
            # unrecoverable; respawn turns that exit into a recovery attempt.
            #
            # The delay is deliberately long. nvargus-daemon garbage-collects a dead
            # client's session slowly, and a 2 s respawn was observed to outrun that
            # cleanup: every doomed attempt left another half-created session behind,
            # digging the daemon deeper until only a host-side restart recovered it.
            respawn=True,
            respawn_delay=15.0,
        ),
    ])
