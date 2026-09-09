# Tvarometr - Automated Face Analysis and Robot Drawing System

An automated system for capturing, analyzing, and robotically rendering human
faces at public events. Computer vision and neural networks detect a face and
predict age, gender and emotion; the result becomes a trajectory that an ABB
GoFa robot draws over the RWS 2.0 interface.

Built for events such as open days at Brno University of Technology, Faculty of
Mechanical Engineering.

> Being rebuilt on branch `rebuild-docker-bt` towards a fully Dockerized system
> with a BehaviorTree.CPP orchestrator. Development containers are the only
> supported mode right now - there is no production image.

**Česky: [rychlý start pro vývoj](VYVOJ.md).**

## Workflow

1. Webcam captures an image of a person
2. Neural networks analyze face attributes (age, gender, emotion)
3. The system generates a drawing trajectory from the analysis
4. The trajectory goes to the ABB GoFa over RWS 2.0
5. The robot draws

## Architecture

### ROS2 packages

#### `tvarometr_inference`
Image acquisition and neural network inference.

- **Nodes:** `usb_cam` (from the `usb_cam` package) streams the webcam,
  configured in `config/usb_cam.yaml`; `inference_node_exec` keeps the newest
  frame and runs the models when triggered; `drawing_node_exec` turns a face
  analysis into the path to draw, over the `drawing_node/generate_drawing`
  action - a managed node, and the launch file does not start it.
- **Topics:** `/image_raw` (sensor_msgs/Image) camera stream,
  `/start_inference` (std_msgs/String) trigger,
  `/face_attributes` (std_msgs/String) JSON with age, gender and emotion.

#### `robot_control`
The ABB robot driver, a managed node speaking Robot Web Services.

- **Nodes:** `robot_controller_node_exec` - RWS session, motion actions, joint
  states.
- **Interfaces:** `robot_robtarget_move` / `robot_jointtarget_move` (actions)
  stream a path into the RAPID buffer queue over DIPC; `controller_request`
  (service) calls one of the RWS methods listed in `commands.py`, which the
  `help` command names; `joint_states` (sensor_msgs/JointState) reports the
  robot pose while active.

#### `master_pkg` (reference only)
The pre-rebuild system: state machine, its own RWS client, path generation and
the turtlesim preview. Not built into any container - it is kept as the
reference for reimplementing the orchestrator and no longer runs as part of the
stack.

### Neural network models

Kept in `models/` at the repo root, out of the source tree - they are half a
gigabyte between them, stored via Git LFS. The inference node takes a
`models_dir` parameter, `/opt/tvarometr/models` inside the container.

- `yolov8x_person_face.pt` - face detection (YOLOv8)
- `model_imdb_cross_person_4.22_99.46.pth.tar` - age estimation
- `affectnet7_model.pth` - emotion classification

### Python libraries

Pinned in `docker/requirements-vision.txt` and `docker/requirements-control.txt`.
A few of those pins are load-bearing - the files explain which and why.

## Getting started

### Requirements

- Ubuntu 22.04 with a native Docker Engine. Not Docker Desktop: it runs
  containers in a VM and cannot pass the host GPU through on Linux. Check that
  `docker context ls` shows `default` as active.
- NVIDIA Container Toolkit, for the vision container only.
- An ABB GoFa with RWS 2.0, to drive a real robot.

ROS2 Humble, Python and every model dependency live in the containers, so
nothing else is needed on the host.

### First run

```bash
git clone https://github.com/Katzoun/tvarometr_ws.git
cd tvarometr_ws
git lfs install && git lfs pull      # weights; LFS pointers are ~130 bytes
docker compose -f docker-compose.dev.yml up -d --build control vision
```

Put the robot's address and credentials in
`src/robot_control/config/robot_control.yaml` before the first run.

`docker-compose.dev.yml` is the only compose file. It defines two environments
that share the source tree but keep their own dependencies:

| Service | Container | Needs | Owns |
| --- | --- | --- | --- |
| `control` | `tvarometr_control_dev` | CPU only | `robot_control_msgs`, `robot_control` |
| `vision` | `tvarometr_vision_dev` | NVIDIA GPU | `tvarometr_interfaces`, `tvarometr_inference` |

`vision` sits behind a Compose profile, so a plain `up -d` starts control alone;
naming the service explicitly starts it. Both bind-mount the repo at
`/workspace` and then idle - they never launch a node by themselves, so you
start one from a terminal and stop it with Ctrl+C.

### VS Code

Open the repo on the host, run **Dev Containers: Reopen in Container** and pick
**robot_control** or **vision**. That service starts, its packages build
automatically, and every terminal in the window runs inside the container. To
work on both at once, open the repo in a second window and pick the other one;
closing a window leaves the containers running. See the
[VS Code multi-container workflow](https://code.visualstudio.com/remote/advancedcontainers/connect-multiple-containers).

### Command line

```bash
docker compose -f docker-compose.dev.yml exec control bash   # or: exec vision bash
```

Every shell sources ROS through `docker/ros-env.sh`. Build once after the
container is created:

```bash
# in control
colcon build --symlink-install --packages-select robot_control_msgs robot_control
# in vision
colcon build --symlink-install --packages-select tvarometr_interfaces tvarometr_inference

source /opt/colcon_ws/install/setup.bash
```

Stop both environments from the host:

```bash
docker compose -f docker-compose.dev.yml --profile vision stop
```

## Running the system

### Robot driver

In a control terminal:

```bash
ros2 launch robot_control robot_control.launch.py
```

The driver is a managed node and comes up `unconfigured`, doing nothing until it
is driven through the lifecycle from a second terminal:

```bash
ros2 lifecycle set /robot_controller configure
ros2 lifecycle set /robot_controller activate
```

Everything else on the controller goes through one service, which lists itself:

```bash
ros2 service call /robot_controller/controller_request \
    robot_control_msgs/srv/RobotRequestSrv "{command: 'help'}" | sed 's/\\n/\n/g'
```

The `sed` is there only because `ros2 service call` prints the response on a
single line. `help` answers before `configure` as well - it never touches the
robot. Parameters go in as `name=value`, in any order:

```bash
ros2 service call /robot_controller/controller_request \
    robot_control_msgs/srv/RobotRequestSrv \
    "{command: 'get_rapid_symbol', params: ['symbol_name=current_state', 'module_name=TRobMain']}"
```

Only what `src/robot_control/robot_control/commands.py` lists can be called, and
a command that changes something is refused while a trajectory is running.

**There is no orchestrator yet.** Until the BehaviorTree one exists, send goals
by hand:

```bash
ros2 action send_goal /robot_controller/robot_robtarget_move \
    robot_control_msgs/action/ExecutePoseArray \
    "{motion_command: MoveL, speed: '100', path: {poses: [...]}}"
```

### Vision

In a vision terminal:

```bash
ros2 launch tvarometr_inference vision.launch.py device:=cuda:0 use_camera:=false
```

This loads the models and waits for images and a trigger. Two more vision
terminals watch the result and fire the trigger:

```bash
ros2 topic echo /face_attributes
ros2 topic pub --once /start_inference std_msgs/msg/String '{data: start}'
```

To use a webcam, set `CAMERA_DEVICE` in `.env` before creating the container and
launch with `use_camera:=true`. Models are mounted read-only from `models/`, so
swapping weights needs only a node restart, not an image rebuild.

Quick GPU check:

```bash
python3 -c 'import torch; assert torch.cuda.is_available(); print(torch.cuda.get_device_name(0))'
```

## Working on the code

Python edits take effect when you restart the node - `--symlink-install` makes
the installed package point back at the source. Rebuild only after changing
message or action definitions, entry points, or installed launch and config
files. When a shared interface changes, rebuild every consumer: vision owns
`tvarometr_interfaces`, control owns `robot_control_msgs`.

Both containers use host networking, shared IPC and the same `ROS_DOMAIN_ID`, so
nodes in either one discover each other.

`colcon test --packages-select robot_control` checks that the command table
still matches the methods it names. It needs neither a robot nor a rebuild.

### Where the files are

Colcon writes outside the bind mount, so build output never lands on the host:

```text
/workspace/src/          source, shared with the host
/opt/colcon_ws/build/    inside the container
/opt/colcon_ws/install/  inside the container
/opt/colcon_ws/log/      inside the container
```

The paths come from `docker/colcon-defaults.yaml`. They are not in a volume:
build output survives stopping and starting a container and is discarded when
the container is rebuilt, so every rebuild starts from a clean install. Named
volumes cover only editor extensions and assistant settings, which are
expensive to reinstall.

### Rebuilding an image

After a change to a requirements file or a Dockerfile, use **Dev Containers:
Rebuild Container**, or on the host:

```bash
docker compose -f docker-compose.dev.yml up -d --build control vision
```

### Which file does what

| File | Role |
| --- | --- |
| `docker/control.Dockerfile`, `docker/vision.Dockerfile` | Dependencies for each container. No source, models or prebuilt workspace. |
| `docker-compose.dev.yml` | How the containers run: mounts, GPU, camera, network. |
| `.devcontainer/control/`, `.devcontainer/vision/` | Which service VS Code attaches to, its extensions, and the build on create. |
| `docker/colcon-defaults.yaml` | Colcon output paths. |
| `docker/ros-env.sh` | Sources ROS and the built workspace in every shell. |
| `docker/entrypoint.sh` | Sources ROS, then runs the container command. |

## Configuration

### Robot

Address and credentials come from `src/robot_control/config/robot_control.yaml`,
which the launch file passes to the driver as ROS parameters:
`connection.ip_address`, `connection.port`, `connection.username`,
`connection.password`. The virtual controller usually listens on port 80, the
physical one on 443.

The file is re-read on every `configure`, so the driver can be pointed at a
different controller without restarting the process: `cleanup`, edit the YAML,
`configure` again. That reload also overwrites anything set with
`ros2 param set` since the last `configure`.

### Environment

Optional `.env` in the repo root, read by Compose:

| Variable | Default | Effect |
| --- | --- | --- |
| `ROS_DOMAIN_ID` | `42` | DDS domain shared by both containers |
| `CAMERA_DEVICE` | `/dev/null` | Host webcam passed to vision as `/dev/video0` |

## Project structure

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
│   ├── robot_control_msgs/        # driver msg/srv/action definitions
│   ├── robot_control/             # ABB robot driver, managed node (RWS)
│   │   ├── launch/robot_control.launch.py
│   │   ├── config/robot_control.yaml  # address, credentials, rates
│   │   ├── test/                  # command table vs. the code it names
│   │   └── robot_control/
│   │       ├── robot_controller_node.py
│   │       ├── commands.py        # what the request service may call
│   │       ├── conversions.py     # ROS messages <-> RAPID literals
│   │       ├── constants.py       # names shared with the RAPID program
│   │       └── rws/               # HTTP client, RWS calls
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
├── .devcontainer/                 # VS Code configs for control/ and vision/
└── docker-compose.dev.yml         # control + optional GPU vision, source mounted
```

## The pre-rebuild system

`master_pkg` drove the whole pipeline before the rebuild. Its state machine ran
IDLE → CAPTURE → INFERENCE → TRAJECTORY_GENERATION → ROBOT_EXECUTION → COMPLETE,
with a multi-line trajectory generator turning face analysis into a drawable
path. It is reference material for the BehaviorTree orchestrator, not part of
any container.

Running it needs a host ROS2 install, including `ros-humble-usb-cam`:

```bash
git lfs install && git lfs pull
colcon build
source install/setup.bash

ros2 launch tvarometr_inference vision.launch.py \
    device:=cuda:0 models_dir:=$PWD/models     # terminal 1
ros2 launch master_pkg control.launch.py       # terminal 2
ros2 run master_pkg keyboard_publisher_exec    # terminal 3
```

## Institution

Developed at Brno University of Technology (BUT)  
Faculty of Mechanical Engineering (FME)  
Brno, Czech Republic
