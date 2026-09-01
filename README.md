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

#### `tvarometr_inference`
Image acquisition and neural network inference.
- **Nodes:**
  - `usb_cam` (from the `usb_cam` package): streams the webcam, configured in
    `config/usb_cam.yaml`
  - `inference_node_exec`: keeps the newest frame and runs the models when
    triggered

- **Topics:**
  - `/image_raw` (sensor_msgs/Image): camera stream
  - `/start_inference` (std_msgs/String): trigger from the master node
  - `/face_attributes` (std_msgs/String): JSON with age, gender and emotion

#### `tvarometr_robot_control`
The ABB robot driver, a managed node speaking Robot Web Services.
- **Nodes:**
  - `robot_controller_node_exec`: RWS session, motion actions, joint states

- **Interfaces:**
  - `robot_robtarget_move` / `robot_jointtarget_move` (actions): stream a path
    into the RAPID buffer queue over DIPC
  - `controller_request` (service): call an RWS method by name
  - `joint_states` (sensor_msgs/JointState): the robot pose while active

#### `master_pkg` (reference only)
The pre-rebuild system: state machine, its own RWS client, path generation and
the turtlesim preview. **Not built into any container** - it is kept in the repo
as the reference for reimplementing the orchestrator, and it no longer runs as
part of the stack.

### Neural Network Models

Kept in `models/` at the repo root, out of the source tree - they are half a
gigabyte between them. The inference node takes a `models_dir` parameter
(`/opt/tvarometr/models` inside the container):
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

`ROBOT_BACKEND=sim` in `.env` (the default) runs the driver's stand-in
controller, which accepts every motion goal and keeps the targets it was sent.
`rws` talks to a real or virtual ABB controller.

The control container now runs the robot driver on its own. It is a managed
node, so it comes up unconfigured and does nothing until it is driven through
the lifecycle:

```bash
docker exec tvarometr_control ros2 lifecycle set /robot_controller configure
docker exec tvarometr_control ros2 lifecycle set /robot_controller activate
```

**There is no orchestrator yet.** `master_pkg` used to be it, and it is no
longer part of any container - it stays in the repo purely as the reference to
reimplement from. Until the BehaviorTree orchestrator exists, the driver has to
be driven by hand:

```bash
ros2 action send_goal /robot_controller/robot_robtarget_move \
    tvarometr_robot_control_msgs/action/ExecutePoseArray \
    "{motion_command: MoveL, speed: '100', path: {poses: [...]}}"
```

**Note:** the vision container needs a native Docker Engine (not Docker Desktop,
which runs containers in a VM and cannot pass through the host GPU on Linux) plus
the NVIDIA Container Toolkit. Check with `docker context ls` that the `default`
context is active.

### Working on the code

Rebuilding the image for every edit is slow. Copy
`docker-compose.override.yml.example` to `docker-compose.override.yml` (git-ignored)
to mount the workspace source into the containers, then rebuild in place:

```bash
docker compose exec vision bash -lc \
  "cd /workspace && colcon build --packages-select tvarometr_inference"
docker compose restart vision
```

Every Bash shell in either image sources the ROS distribution and the workspace
automatically. For example, the development control shell is simply:

```bash
docker compose exec control bash
```

The shared setup lives in `docker/ros-env.sh`; the image entrypoint and Bash's
startup files both use it, so interactive and scripted `bash` invocations see
the same environment.

That takes a couple of seconds instead of several minutes.

**When you change something in `tvarometr_interfaces`, build both images** -
`docker compose build` with no service name. Rebuilding only one leaves the
other with the old definitions, and the containers then discover each other but
cannot make sense of what the other sends, which does not look like the actual
cause at all. Both images carry a fingerprint of the definitions they were built
from and print it on startup, so a mismatch is one command away:

```bash
docker compose exec vision cat /etc/tvarometr_interfaces.sha
docker compose exec control cat /etc/tvarometr_interfaces.sha
```

In the mounted dev setup the same applies inside the containers - rebuild
`tvarometr_interfaces` in both, not just the one you are working on.

### Bare-metal (legacy, pre-rebuild)

This runs the old `master_pkg` pipeline outside Docker. Kept for reference -
the containers no longer carry `master_pkg`.

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

4. Launch (needs `ros-humble-usb-cam` installed as well):
```bash
source install/setup.bash
ros2 launch tvarometr_inference vision.launch.py \
    device:=cuda:0 models_dir:=$PWD/models              # terminal 1
ros2 launch master_pkg control.launch.py             # terminal 2
ros2 run master_pkg keyboard_publisher_exec          # terminal 3
```


### Robot Configuration

Address and credentials come from `.env` (see `.env.example`) and are passed to
the driver as ROS parameters - `connection.ip_address`, `connection.port`,
`connection.username`, `connection.password`. The virtual controller usually listens on port 80, the
physical one on 443.

Running without a robot at all: `ROBOT_BACKEND=sim`.

## Project Structure

```
tvarometr_ws/
├── src/
│   ├── tvarometr_inference/       # Camera + the three networks (GPU container)
│   │   ├── launch/vision.launch.py
│   │   ├── config/usb_cam.yaml    # resolution, framerate, device path
│   │   └── tvarometr_inference/
│   │       ├── inference_node.py
│   │       └── vendor/            # MiVOLO and ResEmoteNet, vendored as-is
│   ├── tvarometr_interfaces/      # msg/srv/action definitions
│   ├── tvarometr_robot_control_msgs/  # driver msg/srv/action definitions
│   ├── tvarometr_robot_control/   # ABB robot driver, managed node (RWS)
│   │   ├── launch/robot_control.launch.py
│   │   └── tvarometr_robot_control/
│   │       ├── robot_controller_node.py
│   │       └── rws/               # HTTP client, RWS calls, sim stand-in
│   └── master_pkg/                # Pre-rebuild system, reference only
│       ├── launch/control.launch.py
│       └── master_pkg/
│           ├── master.py
│           └── utils/
│               ├── state_machine.py
│               ├── rwsprovider.py
│               ├── rwswrappers.py
│               └── path_generator_multiline.py
├── models/                        # Network weights, Git LFS
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
colcon build
source install/setup.bash
```
## Institution

Developed at Brno University of Technology (BUT)  
Faculty of Mechanical Engineering (FME)  
Brno, Czech Republic
