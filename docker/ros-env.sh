#!/bin/bash
# Shared ROS environment for container entrypoints and interactive shells.

if [[ "${ROS_ENV_SOURCED:-}" != "1" ]]; then
    source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"

    if [[ -f /opt/colcon_ws/install/setup.bash ]]; then
        source /opt/colcon_ws/install/setup.bash
    fi

    export ROS_ENV_SOURCED=1
fi
