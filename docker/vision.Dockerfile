# Vision container: webcam capture + age/gender/emotion inference (GPU).
# Dependencies only - source and model weights are mounted by Compose.

# "base" rather than "cudnn8-runtime": the torch wheels ship their own cuBLAS
# and cuDNN, so a fuller CUDA image duplicates ~3 GB.
FROM nvidia/cuda:12.1.1-base-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=humble \
    LANG=en_US.UTF-8 \
    COLCON_DEFAULTS_FILE=/colcon-defaults.yaml \
    BASH_ENV=/ros-env.sh \
    # ultralytics pip-installs missing deps on first model load - no use at an
    # event with no network, so fail the build instead.
    YOLO_AUTOINSTALL=false \
    # /root/.config isn't writable here and ultralytics would relocate anyway
    YOLO_CONFIG_DIR=/tmp/Ultralytics

# --- Base OS + locale --------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        locales curl gnupg2 lsb-release software-properties-common ca-certificates \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

# --- ROS2 Humble --------------------------------------------------------
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
        > /etc/apt/sources.list.d/ros2.list \
    && apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-ros-base \
        ros-humble-cv-bridge \
        ros-humble-usb-cam \
        ros-humble-sensor-msgs \
        ros-humble-geometry-msgs \
        ros-humble-std-msgs \
        python3-colcon-common-extensions \
        python3-pip \
        build-essential \
        cmake \
        git \
        git-lfs \
        # opencv runtime libs
        libgl1 libglib2.0-0 libgomp1 \
    && rm -rf /var/lib/apt/lists/*

# --- Python dependencies -------------------------------------------------
# Upgrade together: apt ships packaging 21.3, too old for the setuptools that
# torch pulls in, and the mismatch breaks colcon with canonicalize_version().
RUN pip3 install --no-cache-dir --upgrade pip setuptools packaging

COPY docker/requirements-vision.txt /tmp/requirements-vision.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements-vision.txt

WORKDIR /workspace
COPY docker/colcon-defaults.yaml /colcon-defaults.yaml
COPY docker/ros-env.sh /ros-env.sh
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh /ros-env.sh \
    && echo 'source /ros-env.sh' >> /root/.bashrc

ENTRYPOINT ["/entrypoint.sh"]
CMD ["sleep", "infinity"]
