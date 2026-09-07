#!/usr/bin/env python3
"""Starts the robot controller.

It is a managed node, so it comes up unconfigured and touches nothing until it
is driven through the lifecycle:

    ros2 lifecycle set /robot_controller configure
    ros2 lifecycle set /robot_controller activate

Configure is what opens the session to the controller, so it is also where a
wrong address or a robot that is switched off shows up - as a failed
transition, not a crashed node.

All settings come from the YAML below, which is installed into the image. There
are no launch arguments for the connection: the robot's configuration belongs
to the config file, not to the environment.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory("robot_control"), "config", "robot_control.yaml"
    )

    config = LaunchConfiguration("config")

    robot_controller_node = LifecycleNode(
        package="robot_control",
        executable="robot_controller_node_exec",
        name="robot_controller",
        namespace="",
        output="screen",
        parameters=[config],
        emulate_tty=True,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value=default_config,
                description="YAML with the connection, keepalive and DIPC retry settings",
            ),
            LogInfo(msg="Starting robot controller..."),
            robot_controller_node,
        ]
    )
