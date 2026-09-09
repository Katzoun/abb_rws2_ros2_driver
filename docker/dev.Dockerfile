FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=humble \
    COLCON_DEFAULTS_FILE=/colcon-defaults.yaml \
    BASH_ENV=/ros-env.sh

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-colcon-common-extensions \
        python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Manifests only: the source itself is mounted at run time.
COPY robot_control/package.xml /tmp/deps/robot_control/package.xml
COPY robot_control_msgs/package.xml /tmp/deps/robot_control_msgs/package.xml
RUN apt-get update \
    && rosdep update --rosdistro ${ROS_DISTRO} \
    && rosdep install --from-paths /tmp/deps --ignore-src --rosdistro ${ROS_DISTRO} -y \
    && rm -rf /tmp/deps /var/lib/apt/lists/*

WORKDIR /workspace
COPY docker/colcon-defaults.yaml /colcon-defaults.yaml
COPY docker/ros-env.sh /ros-env.sh
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh /ros-env.sh \
    && echo 'source /ros-env.sh' >> /root/.bashrc

ENTRYPOINT ["/entrypoint.sh"]
CMD ["sleep", "infinity"]
