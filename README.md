# ABB RWS driver for ROS 2

A ROS 2 driver for ABB robots over Robot Web Services 2.0. It exposes an ABB
controller as a managed node: two actions stream a path into the RAPID buffer
queue, one service reaches the rest of RWS, and joint states are published while
the node is active.

> Under active rebuild. Development containers are the only supported mode right
> now - there is no production image.

See the [development quick start](DEVELOPMENT.md) for the short version.

## Architecture

### `robot_control`

The driver itself, a managed (lifecycle) node speaking RWS.

- **Node:** `robot_controller_node_exec` - RWS session, motion actions, joint
  states.
- **Interfaces:** `robot_robtarget_move` / `robot_jointtarget_move` (actions)
  stream a path into the RAPID buffer queue over DIPC; `controller_request`
  (service) calls one of the RWS methods listed in `commands.py`, which the
  `help` command names; `joint_states` (sensor_msgs/JointState) reports the
  robot pose while active.

### `robot_control_msgs`

The driver's interface definitions: `ExecutePoseArray` and `ExecuteJointArray`
actions, the `RobotRequestSrv` service and the `RobotJoints` message. They live
here rather than in a repo of their own because they version with the driver;
a consumer that only needs to talk to the driver builds this package alone.

### Python libraries

Pinned in `docker/requirements-control.txt`. The driver needs `requests` and
what ROS 2 Humble already brings.

## Getting started

### Requirements

- Ubuntu 22.04 with a native Docker Engine. Check that `docker context ls`
  shows `default` as active.
- An ABB robot with RWS 2.0 - a physical controller or RobotStudio's virtual
  one.

ROS 2 Humble and every Python dependency live in the container, so nothing else
is needed on the host.

### First run

```bash
git clone https://github.com/Katzoun/abb_rws2_ros2_driver.git
cd abb_rws2_ros2_driver
docker compose -f docker-compose.dev.yml up -d --build
```

Put the robot's address and credentials in `robot_control/config/robot_control.yaml`
before the first run.

The container bind-mounts the repo at `/workspace` and then idles - it never
launches a node by itself, so you start one from a terminal and stop it with
Ctrl+C.

### VS Code

Open the repo on the host and run **Dev Containers: Reopen in Container**. The
service starts, the packages build automatically, and every terminal in the
window runs inside the container. Closing the window leaves the container
running.

### Command line

```bash
docker compose -f docker-compose.dev.yml exec control bash
```

Every shell sources ROS through `docker/ros-env.sh`. Build once after the
container is created:

```bash
colcon build --symlink-install --packages-select robot_control_msgs robot_control
source /opt/colcon_ws/install/setup.bash
```

Stop the container from the host:

```bash
docker compose -f docker-compose.dev.yml stop
```

## Running the driver

In a container terminal:

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

Only what `robot_control/robot_control/commands.py` lists can be called, and a
command that changes something is refused while a trajectory is running.

Motion goals go in as actions:

```bash
ros2 action send_goal /robot_controller/robot_robtarget_move \
    robot_control_msgs/action/ExecutePoseArray \
    "{motion_command: MoveL, speed: '100', path: {poses: [...]}}"
```

## Using it from another workspace

The packages sit at the repo root, so colcon finds them wherever the repo is
cloned inside a workspace's `src/`. The ROS-native way is a `.repos` file:

```yaml
repositories:
  abb_rws2_ros2_driver:
    type: git
    url: https://github.com/Katzoun/abb_rws2_ros2_driver.git
    version: main
```

```bash
vcs import src < driver.repos
colcon build --packages-select robot_control_msgs   # a consumer that only talks to the driver
```

Add `src/abb_rws2_ros2_driver/` to the consuming workspace's `.gitignore`.

## Working on the code

Python edits take effect when you restart the node - `--symlink-install` makes
the installed package point back at the source. Rebuild only after changing
message or action definitions, entry points, or installed launch and config
files.

`colcon test --packages-select robot_control` runs the test suite, which needs
neither a robot nor a rebuild.

### Where the files are

Colcon writes outside the bind mount, so build output never lands on the host:

```text
/workspace/              source, shared with the host
/opt/colcon_ws/build/    inside the container
/opt/colcon_ws/install/  inside the container
/opt/colcon_ws/log/      inside the container
```

The paths come from `docker/colcon-defaults.yaml`. They are not in a volume:
build output survives stopping and starting the container and is discarded when
the container is rebuilt, so every rebuild starts from a clean install. Named
volumes cover only editor extensions and assistant settings, which are
expensive to reinstall.

### Rebuilding the image

After a change to the requirements file or the Dockerfile, use **Dev Containers:
Rebuild Container**, or on the host:

```bash
docker compose -f docker-compose.dev.yml up -d --build
```

### Which file does what

| File | Role |
| --- | --- |
| `docker/control.Dockerfile` | Dependencies. No source or prebuilt workspace. |
| `docker-compose.dev.yml` | How the container runs: mounts, network. |
| `.devcontainer/` | Which service VS Code attaches to, its extensions, and the build on create. |
| `docker/colcon-defaults.yaml` | Colcon output paths. |
| `docker/ros-env.sh` | Sources ROS and the built workspace in every shell. |
| `docker/entrypoint.sh` | Sources ROS, then runs the container command. |

## Configuration

### Robot

Address and credentials come from `robot_control/config/robot_control.yaml`,
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
| `ROS_DOMAIN_ID` | `42` | DDS domain the driver joins |

## Project structure

```
abb_rws2_ros2_driver/
├── robot_control_msgs/            # msg/srv/action definitions
├── robot_control/                 # the driver, a managed node (RWS)
│   ├── launch/robot_control.launch.py
│   ├── config/robot_control.yaml  # address, credentials, rates
│   ├── test/
│   └── robot_control/
│       ├── robot_controller_node.py
│       ├── commands.py            # what the request service may call
│       ├── conversions.py         # ROS messages <-> RAPID literals
│       ├── constants.py           # names shared with the RAPID program
│       └── rws/                   # HTTP client, RWS calls
├── docker/                        # Dockerfile, pinned requirements, entrypoint
├── .devcontainer/                 # VS Code config
└── docker-compose.dev.yml         # the dev container, source mounted
```

## Institution

Developed at Brno University of Technology (BUT)  
Faculty of Mechanical Engineering (FME)  
Brno, Czech Republic
