"""Bring up the whole stack in one command.

    ros2 launch jetson_bringup bringup.launch.py

Starts the camera, the TensorRT detector, and the operator console as one launch
graph, with the console showing the detector's overlay. Individual pieces can be
disabled:

    ros2 launch jetson_bringup bringup.launch.py detector:=false
    ros2 launch jetson_bringup bringup.launch.py console:=false

Everything runs in one container, so `./docker/run.sh` then this command is the
entire startup sequence.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def _include(package, launch_file, condition=None, arguments=None):
    """Include a launch file with its own arguments, isolated from siblings.

    scoped=True is load-bearing. Every one of these launch files declares a
    `params_file` argument, and DeclareLaunchArgument does NOT apply its default when
    the name already exists in the enclosing context -- so without a scope the first
    include's value silently leaks into the rest. Observed: trt_detector started with
    the camera's parameter file.

    forwarding stays at its default (True): scoping alone stops values escaping to
    siblings, whereas forwarding=False also hides the PARENT's arguments, which broke
    the `detector`/`console` switches with "launch configuration does not exist".
    """
    return GroupAction(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory(package), 'launch', launch_file)
                ),
                launch_arguments=arguments or {},
            )
        ],
        condition=condition,
        scoped=True,
    )


def generate_launch_description():
    detector = LaunchConfiguration('detector')
    console = LaunchConfiguration('console')

    # With the detector running the console should show boxes; without it, the raw
    # camera. Resolved at launch time so one argument does the right thing either way.
    console_topic = PythonExpression([
        "'/trt_detector/overlay' if '", detector, "' == 'true' else '/image_raw'"
    ])

    return LaunchDescription([
        DeclareLaunchArgument('detector', default_value='true',
                              description='run the TensorRT detector'),
        DeclareLaunchArgument('console', default_value='true',
                              description='run the web operator console'),

        _include('jetson_camera', 'camera.launch.py'),
        _include('trt_detector', 'detector.launch.py',
                 condition=IfCondition(detector)),
        _include('operator_console', 'console.launch.py',
                 condition=IfCondition(console),
                 arguments={'image_topic': console_topic}.items()),
    ])
