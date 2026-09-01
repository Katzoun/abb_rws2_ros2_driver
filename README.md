# Tvarometr - Automated Face Analysis and Robot Drawing System

An automated system for capturing, analyzing, and robotically rendering human faces at public events. Uses computer vision and neural networks to detect faces, predict age, gender, and emotion, then generates trajectory paths for ABB GoFa robot execution via RWS 2.0 interface.

## Overview

This ROS2 system integrates webcam capture, deep learning inference, and industrial robot control to create an interactive installation for events such as open days at Brno University of Technology, Faculty of Mechanical Engineering (BUT FME).

**Workflow:**
1. Webcam captures image of person
2. Neural networks analyze face attributes (age, gender, emotion)
3. System generates drawing trajectory based on analysis
4. Trajectory sent to ABB GoFa robot via RWS 2.0 protocol
5. Robot executes drawing

## System Requirements

- **OS**: Ubuntu 22.04 LTS
- **ROS**: ROS2 Humble
- **Python**: 3.10+
- **Robot**: ABB GoFa with RWS 2.0 support
- **Hardware**: Webcam (USB or integrated)

## Architecture

### ROS2 Packages

#### `camera`
Handles image acquisition and neural network inference.
- **Nodes:**
  - `camera_node_exec`: Captures frames from webcam, publishes on `/input_image` topic
  - `inference_node_exec`: Runs deep learning models for face attribute prediction

- **Topics:**
  - `/start_inference` (std_msgs/String): Trigger for image capture
  - `/input_image` (sensor_msgs/Image): Raw camera frames
  - `/inference_result` (custom): Face analysis results (age, gender, emotion)

#### `master_pkg`
Orchestrates system state machine and robot communication.
- **Nodes:**
  - `master_node`: State machine controller, RWS client, trajectory generator

- **Features:**
  - State machine for process workflow
  - ABB Robot Web Services (RWS 2.0) client
  - Path generation and RAPID code generation
  - Trajectory optimization for robot execution

### Neural Network Models

Located in `src/camera/camera/AgeGenderEmotionPrediction/models/`:
- `yolov8x_person_face.pt`: Face detection (YOLOv8)
- `model_imdb_cross_person_4.22_99.46.pth.tar`: Age estimation
- `affectnet7_model.pth`: Emotion classification

Models stored via Git LFS due to file size constraints.


### Python Libraries

Pinned in `docker/requirements-vision.txt` (inference) and
`docker/requirements-control.txt` (robot control). A few of those pins are
load-bearing - the files explain which and why.

## Installation & Usage

> The project is being rebuilt on branch `rebuild-docker-bt` towards a fully
> Dockerized system with a BehaviorTree.CPP orchestrator, replacing the setup
> below. See the plan for details. This README still describes the current
> (pre-rebuild) system.

### Docker (recommended, Phase 1 of the rebuild)

Runs the existing system unchanged in two containers (GPU vision, CPU control) -
no host ROS2 install required beyond Docker + the NVIDIA Container Toolkit.

```bash
cd ~
git clone https://github.com/Katzoun/tvarometr_ws.git
cd tvarometr_ws
git lfs install && git lfs pull   # required before building the vision image

cp .env.example .env              # edit robot credentials / device settings
docker compose build
docker compose up
```

`USE_RWS=false` in `.env` (the default) runs the turtlesim preview instead of
talking to a real robot.

Keyboard control (s/r/q/e) runs as its own process with its own terminal, the
same way the bare-metal setup did - start it in a second terminal once the stack
is up:

```bash
docker exec -it tvarometr_control ros2 run master_pkg keyboard_publisher_exec
```

**Note:** the vision container needs a native Docker Engine (not Docker Desktop,
which runs containers in a VM and cannot pass through the host GPU on Linux) plus
the NVIDIA Container Toolkit. Check with `docker context ls` that the `default`
context is active.

### Bare-metal (legacy)

1. Clone repository:
```bash
cd ~
git clone https://github.com/Katzoun/tvarometr_ws.git
cd tvarometr_ws
```

2. Install Git LFS and pull models:
```bash
git lfs install
git lfs pull
```

3. Build ROS2 workspace:
```bash
colcon build
source install/setup.bash
```

4. Launch:
```bash
source install/setup.bash
ros2 launch master_pkg tvarometr_system.launch.py
```

Or run each node manually in separate terminals:
```bash
ros2 run camera camera_node_exec
ros2 run camera inference_node_exec
ros2 run master_pkg master_node_exec
ros2 run master_pkg keyboard_publisher_exec
```


### Robot Configuration

Address and credentials come from `.env` (see `.env.example`) and are passed to
the master node as ROS parameters - `robot_ip`, `robot_port`, `robot_username`,
`robot_password`. The virtual controller usually listens on port 80, the
physical one on 443.

Running without a robot at all: `USE_RWS=false`.

## Project Structure

```
tvarometr_ws/
├── src/
│   ├── camera/                    # Vision and inference package
│   │   ├── launch/
│   │   │   └── vision.launch.py   # camera + inference (vision container)
│   │   └── camera/
│   │       ├── camera_controller.py
│   │       └── AgeGenderEmotionPrediction/
│   │           ├── face_attributes_node.py
│   │           ├── predict.py
│   │           └── models/         # Neural network weights (Git LFS)
│   └── master_pkg/                # Control and coordination package
│       ├── launch/
│       │   ├── control.launch.py  # master + turtle preview (control container)
│       │   └── tvarometr_system.launch.py
│       └── master_pkg/
│           ├── master.py
│           └── utils/
│               ├── state_machine.py
│               ├── rwsprovider.py
│               ├── rwswrappers.py
│               └── path_generator_multiline.py
├── docker/                        # Dockerfiles, pinned requirements, entrypoint
├── docker-compose.yml
└── .env.example
```

## Technical Details

### State Machine
Master node implements finite state machine with states:
- IDLE: Awaiting trigger
- CAPTURE: Image acquisition
- INFERENCE: Neural network processing
- TRAJECTORY_GENERATION: Path planning
- ROBOT_EXECUTION: RAPID program upload and execution
- COMPLETE: Process finished

### Path Generation
Multi-line trajectory generator creates robot-executable paths based on face analysis results.

## Development

### Building
```bash
colcon build --packages-select camera master_pkg
source install/setup.bash
```
## Institution

Developed at Brno University of Technology (BUT)  
Faculty of Mechanical Engineering (FME)  
Brno, Czech Republic
