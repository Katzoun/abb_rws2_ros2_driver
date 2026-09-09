# Production image: the build is baked in and the container launches the driver.

FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=humble \
    BASH_ENV=/ros-env.sh

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-colcon-common-extensions \
        nano \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /build
COPY robot_control/package.xml robot_control/package.xml
COPY robot_control_msgs/package.xml robot_control_msgs/package.xml
RUN apt-get update \
    && rosdep update --rosdistro ${ROS_DISTRO} \
    && rosdep install --from-paths /build --ignore-src --rosdistro ${ROS_DISTRO} -y \
    && rm -rf /var/lib/apt/lists/*

COPY robot_control_msgs robot_control_msgs
COPY robot_control robot_control
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && colcon build \
        --packages-select robot_control_msgs robot_control \
        --build-base /build/build \
        --install-base /opt/colcon_ws/install \
        --merge-install \
    && cd / && rm -rf /build
WORKDIR /

COPY docker/ros-env.sh /ros-env.sh
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh /ros-env.sh \
    && echo 'source /ros-env.sh' >> /root/.bashrc

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "robot_control", "robot_control.launch.py"]
