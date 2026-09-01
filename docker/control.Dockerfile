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
COPY src/tvarometr_interfaces src/tvarometr_interfaces
COPY src/tvarometr_core src/tvarometr_core
COPY src/tvarometr_robot_control src/tvarometr_robot_control
COPY src/master_pkg src/master_pkg

RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && colcon build --symlink-install --packages-select tvarometr_interfaces tvarometr_core tvarometr_robot_control master_pkg
# Fingerprint of the interface definitions this image was built from. Both
# images have to agree on it - if they drift, nodes discover each other but the
# messages between them are nonsense, which looks nothing like the actual cause.
RUN find src/tvarometr_interfaces -type f -name '*.msg' \
         -o -type f -name '*.srv' -o -type f -name '*.action' \
    | sort | xargs sha256sum | sha256sum | cut -c1-12 > /etc/tvarometr_interfaces.sha


COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "master_pkg", "control.launch.py"]
