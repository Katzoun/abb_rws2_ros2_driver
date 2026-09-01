# Control container: master node (state machine + RWS client), keyboard input,
# turtle simulator preview. CPU-only, no GPU dependency.
#
# Build context is the repo root (see docker-compose.yml).

FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=humble

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-colcon-common-extensions \
        python3-pip \
        # tkinter backend for the turtle-module drawing preview (use_rws:=false)
        python3-tk \
    && rm -rf /var/lib/apt/lists/*

COPY docker/requirements-control.txt /tmp/requirements-control.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements-control.txt

WORKDIR /workspace
COPY src/tvarometr_robot_control_msgs src/tvarometr_robot_control_msgs
COPY src/tvarometr_robot_control src/tvarometr_robot_control
COPY src/master_pkg src/master_pkg

RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && colcon build --symlink-install --packages-select tvarometr_robot_control_msgs tvarometr_robot_control master_pkg
# One line per interface package: name and a hash of its definitions. Images that
# carry the same package have to agree on its hash - if they drift, nodes discover
# each other but the messages between them are nonsense, which looks nothing like
# the actual cause.
RUN for pkg in src/*_msgs src/tvarometr_interfaces; do \
        [ -d "$pkg" ] || continue; \
        h=$(find "$pkg" -type f -name '*.msg' -o -type f -name '*.srv' \
                -o -type f -name '*.action' | sort | xargs sha256sum \
            | sha256sum | cut -c1-12); \
        echo "$(basename $pkg) $h"; \
    done > /etc/tvarometr_interfaces.sha


COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "master_pkg", "control.launch.py"]
