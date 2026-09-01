#!/bin/bash
set -e

source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"

if [ -f /workspace/install/setup.bash ]; then
    source /workspace/install/setup.bash
fi

# Both containers print this. If the two ever differ, one of the images was
# rebuilt without the other and the nodes will talk past each other.
if [ -f /etc/tvarometr_interfaces.sha ]; then
    echo "tvarometr_interfaces: $(cat /etc/tvarometr_interfaces.sha)"
fi

exec "$@"
