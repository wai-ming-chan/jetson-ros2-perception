"""Bring up the TensorRT detector with the default parameter file."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('trt_detector'), 'config', 'detector_params.yaml'
    )
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        Node(
            package='trt_detector',
            executable='trt_detector_node',
            name='trt_detector',
            parameters=[params_file],
            output='screen',
        ),
    ])
