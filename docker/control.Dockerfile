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
COPY src/robot_control_msgs src/robot_control_msgs
COPY src/robot_control src/robot_control

RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && colcon build --symlink-install --packages-select robot_control_msgs robot_control

COPY docker/ros-env.sh /ros-env.sh
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh /ros-env.sh \
    && echo 'source /ros-env.sh' >> /root/.bashrc

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "robot_control", "robot_control.launch.py"]
