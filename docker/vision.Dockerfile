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
        ros-humble-sensor-msgs \
        ros-humble-geometry-msgs \
        ros-humble-std-msgs \
        python3-colcon-common-extensions \
        python3-pip \
        git \
        git-lfs \
        # opencv-python (non-headless, for the current cv2.imshow preview) runtime libs
        libgl1 libglib2.0-0 libsm6 libxext6 libxrender1 libgomp1 \
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
COPY src/camera src/camera

# Catch LFS pointer files before they end up baked into the image.
RUN for f in \
        src/camera/camera/AgeGenderEmotionPrediction/models/yolov8x_person_face.pt \
        src/camera/camera/AgeGenderEmotionPrediction/models/model_imdb_cross_person_4.22_99.46.pth.tar \
        src/camera/camera/AgeGenderEmotionPrediction/models/affectnet7_model.pth; \
    do \
        size=$(stat -c%s "$f" 2>/dev/null || echo 0); \
        if [ "$size" -lt 1000000 ]; then \
            echo "ERROR: $f is only ${size} bytes - looks like a Git LFS pointer, not the real weight file." >&2; \
            echo "Run 'git lfs install && git lfs pull' in the repo before building this image." >&2; \
            exit 1; \
        fi; \
    done

RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && colcon build --symlink-install --packages-select camera

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "camera", "vision.launch.py"]
