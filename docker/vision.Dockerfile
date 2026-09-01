# Vision container: webcam capture + age/gender/emotion inference (GPU).
# Build context is the repo root, see docker-compose.yml.
#
# Model weights come from Git LFS - run `git lfs pull` before building, or the
# size check below stops you. LFS pointers are ~130 bytes, not real weights.

# "base" tag rather than "cudnn8-runtime": the torch wheels below ship their own
# cuBLAS/cuDNN/etc., so a fuller CUDA image just duplicates ~3 GB of libraries.
# The driver comes from the NVIDIA Container Toolkit at runtime anyway.
FROM nvidia/cuda:12.1.1-base-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=humble \
    LANG=en_US.UTF-8 \
    # ultralytics pip-installs missing deps on first model load, which is no use
    # at an event with no network. Better to fail the build than discover it there.
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
# These two have to be upgraded together. apt ships packaging 21.3, too old for
# the setuptools that torch/ultralytics pull in, and the mismatch later breaks
# colcon's ament_python build with a canonicalize_version() TypeError.
RUN pip3 install --no-cache-dir --upgrade pip setuptools packaging

COPY docker/requirements-vision.txt /tmp/requirements-vision.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements-vision.txt

# --- Workspace source -----------------------------------------------------
WORKDIR /workspace
COPY src/tvarometr_interfaces src/tvarometr_interfaces
COPY src/tvarometr_inference src/tvarometr_inference

# Weights are kept out of the source tree - see models/ in the repo root.
COPY models /opt/tvarometr/models

# Catch LFS pointer files before they end up baked into the image.
RUN for f in /opt/tvarometr/models/*; do \
        size=$(stat -c%s "$f" 2>/dev/null || echo 0); \
        if [ "$size" -lt 1000000 ]; then \
            echo "ERROR: $f is only ${size} bytes - looks like a Git LFS pointer, not the real weight file." >&2; \
            echo "Run 'git lfs install && git lfs pull' in the repo before building this image." >&2; \
            exit 1; \
        fi; \
    done

RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && colcon build --symlink-install --packages-select tvarometr_interfaces tvarometr_inference
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
CMD ["ros2", "launch", "tvarometr_inference", "vision.launch.py"]
