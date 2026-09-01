#!/bin/bash
# Shared ROS environment for container entrypoints and interactive shells.

if [[ "${TVAROMETR_ROS_ENV_SOURCED:-}" != "1" ]]; then
    source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"

    if [[ -f /workspace/install/setup.bash ]]; then
        source /workspace/install/setup.bash
    fi

    export TVAROMETR_ROS_ENV_SOURCED=1
fi
