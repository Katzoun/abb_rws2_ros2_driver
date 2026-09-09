# ABB RWS driver for ROS 2

A ROS 2 driver for ABB robots over Robot Web Services 2.0. It exposes an ABB
controller as a managed node: two actions stream a path into the RAPID buffer
queue, one service reaches the rest of RWS, and joint states are published while
the node is active.

> Under active rebuild. Two ways to run it: a development container with the
> source mounted, and a production image that carries the build and starts the
> driver by itself.

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
actions, the `RobotRequestSrv` service and the `RobotJoints` message. A consumer
that only needs to talk to the driver builds this package alone.

## Getting started

### Requirements

- Ubuntu 22.04 with a native Docker Engine. Check that `docker context ls`
  shows `default` as active.
- An ABB robot with RWS 2.0 - a physical controller or RobotStudio's virtual
  one.

ROS 2 Humble and every Python dependency live in the container, so nothing else
is needed on the host.

### Development container

```bash
git clone https://github.com/Katzoun/abb_rws2_ros2_driver.git
cd abb_rws2_ros2_driver
docker compose -f docker-compose.dev.yml up -d --build
```

Put the robot's address and credentials in `robot_control/config/robot_control.yaml`
first. The container bind-mounts the repo at `/workspace` and then idles - you
start a node yourself and stop it with Ctrl+C. For the deployed container see
[Production](#production).

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

Stop the container from the host with
`docker compose -f docker-compose.dev.yml stop`.

## Running the driver

```bash
ros2 launch robot_control robot_control.launch.py
```

The driver comes up `unconfigured` and does nothing until it is driven through
the lifecycle from a second terminal:

```bash
ros2 lifecycle set /robot_controller configure
ros2 lifecycle set /robot_controller activate
```

Everything else on the controller goes through one service, which lists itself:

```bash
ros2 service call /robot_controller/controller_request \
    robot_control_msgs/srv/RobotRequestSrv "{command: 'help'}" | sed 's/\\n/\n/g'
```

The `sed` is only because `ros2 service call` prints the response on a single
line. `help` answers before `configure` as well. Parameters go in as
`name=value`, in any order:

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

## Production

```bash
docker compose -f docker-compose.prod.yml up -d --build
docker compose -f docker-compose.prod.yml logs -f
```

| | Development | Production |
| --- | --- | --- |
| Source | bind-mounted from the host | copied in at build time |
| Build | `colcon build` by hand | baked into the image |
| Command | `sleep infinity`, you launch the node | `ros2 launch`, the node starts with the container |
| Volumes | editor and assistant state | none |
| Restart | none | `unless-stopped` |

Nothing is mounted, so what runs is what was built. The node still comes up
`unconfigured` - an orchestrator drives it through `configure` and `activate`,
where a wrong address or a robot that is switched off shows up as a failed
transition rather than a crashed container.

The address and credentials are baked in from
`robot_control/config/robot_control.yaml` as it stood at build time. `nano` is
installed so a deployed container can be edited in place; `exec` allocates its
own terminal, so the service needs neither `stdin_open` nor `tty`:

```bash
docker compose -f docker-compose.prod.yml exec driver \
    nano /opt/colcon_ws/install/share/robot_control/config/robot_control.yaml
```

Follow it with a lifecycle `cleanup` and `configure` to reload. The edit
survives `restart` but not `down` or `up --build`, which recreate the container
from the image.

The two compose files can run side by side, but only one of them should hold
the `/robot_controller` name in a given `ROS_DOMAIN_ID`.

## Using it from another workspace

The packages sit at the repo root, so colcon finds them wherever the repo is
cloned inside a workspace's `src/`:

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

A colcon build is not a Docker build: after a change to a `package.xml` or a
Dockerfile, use **Dev Containers: Rebuild Container**, or
`docker compose -f docker-compose.dev.yml up -d --build` on the host.

Dependencies are declared in the two `package.xml` files and installed by
`rosdep` at image build time, so adding one there is enough.

### Where the files are

```text
/workspace/              source, shared with the host
/opt/colcon_ws/build/    inside the container
/opt/colcon_ws/install/  inside the container
/opt/colcon_ws/log/      inside the container
```

The paths come from `docker/colcon-defaults.yaml`. Build output is not in a
volume: it survives stopping and starting the container, and a rebuild starts
from a clean install.

### Which file does what

| File | Role |
| --- | --- |
| `docker/dev.Dockerfile` | Development dependencies. No source or prebuilt workspace. |
| `docker/prod.Dockerfile` | Production image: dependencies, the source and the build. |
| `docker-compose.dev.yml` | How the development container runs: mounts, network. |
| `docker-compose.prod.yml` | How the deployed container runs: no mounts, restarts, launches the node. |
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
`ros2 param set`.

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
├── docker/                        # Dockerfiles, entrypoint, colcon defaults
├── .devcontainer/                 # VS Code config
├── docker-compose.dev.yml         # the dev container, source mounted
└── docker-compose.prod.yml        # the deployed container, source baked in
```

## Institution

Developed at Brno University of Technology (BUT)  
Faculty of Mechanical Engineering (FME)  
Brno, Czech Republic
