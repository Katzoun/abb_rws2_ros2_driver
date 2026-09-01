# Control container: the ABB robot driver (RWS). CPU-only, no GPU dependency.
#
# Build context is the repo root (see docker-compose.yml).

FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=humble \
    BASH_ENV=/ros-env.sh

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-colcon-common-extensions \
        python3-pip \
    && rm -rf /var/lib/apt/lists/*

COPY docker/requirements-control.txt /tmp/requirements-control.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements-control.txt

WORKDIR /workspace
COPY src/tvarometr_robot_control_msgs src/tvarometr_robot_control_msgs
COPY src/tvarometr_robot_control src/tvarometr_robot_control

RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && colcon build --symlink-install --packages-select tvarometr_robot_control_msgs tvarometr_robot_control
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


COPY docker/ros-env.sh /ros-env.sh
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh /ros-env.sh \
    && echo 'source /ros-env.sh' >> /root/.bashrc

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "tvarometr_robot_control", "robot_control.launch.py"]
