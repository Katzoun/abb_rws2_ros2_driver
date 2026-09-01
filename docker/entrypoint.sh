#!/bin/bash
set -e

source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"

if [ -f /workspace/install/setup.bash ]; then
    source /workspace/install/setup.bash
fi

# Interface packages this image was built with. Where two containers carry the
# same package the hashes have to match - if one image was rebuilt without the
# other, the nodes discover each other and then talk past each other.
if [ -f /etc/tvarometr_interfaces.sha ]; then
    while read -r pkg hash; do
        echo "interfaces: $pkg $hash"
    done < /etc/tvarometr_interfaces.sha
fi

exec "$@"
