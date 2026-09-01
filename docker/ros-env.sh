#!/bin/bash
# Shared ROS environment for container entrypoints and interactive shells.

# docker exec starts a new process from the image environment; it does not
# inherit variables that PID 1 exported after the container started. Source the
# ROS setup in every new Bash process, while avoiding duplicate path entries
# when the entrypoint subsequently launches an interactive shell.
if [[ "${TVAROMET_ROS_ENV_SOURCED:-}" != "1" ]]; then
    source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"

    if [[ -f /workspace/install/setup.bash ]]; then
        source /workspace/install/setup.bash
    fi

    export TVAROMET_ROS_ENV_SOURCED=1
fi
