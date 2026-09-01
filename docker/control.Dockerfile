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
COPY src/master_pkg src/master_pkg

RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && colcon build --symlink-install --packages-select master_pkg

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "master_pkg", "control.launch.py"]
