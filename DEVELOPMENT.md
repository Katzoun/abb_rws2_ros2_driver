# Development quick start

The full description is in the [README](README.md#getting-started); this is the
short version.

## 1. Open the environment

On the host, open the repository in VS Code and run **Dev Containers: Reopen in
Container** from `F1`.

VS Code prepares the container and builds the ROS packages. Terminals in that
window then run inside the container. On its own the container starts nothing.

Closing the window does not stop the container; stop it from a host terminal:

```bash
docker compose -f docker-compose.dev.yml stop
```

## 2. Start the node

```bash
ros2 launch robot_control robot_control.launch.py
```

The controller waits in the `unconfigured` state. Bring up the connection to the
robot from a second terminal:

```bash
ros2 lifecycle set /robot_controller configure
ros2 lifecycle set /robot_controller activate
```

## 3. Edit the code

Edit a Python file, stop the node with `Ctrl+C` and run `ros2 launch` again.
Sources are saved straight into the repository on the host.

After changing ROS messages, actions, entry points or launch and config files,
build the packages again and source the result:

```bash
colcon build --symlink-install --packages-select robot_control_msgs robot_control
source /opt/colcon_ws/install/setup.bash
```

Tests: `colcon test --packages-select robot_control`. They need no robot.

After changing dependencies or the Dockerfile, use **Dev Containers: Rebuild
Container**.

## Two different builds

| Operation | What it prepares | When you need it |
| --- | --- | --- |
| Docker build | The image with the system, ROS and the libraries. | The first time, and after a change to dependencies or the Dockerfile. |
| Colcon build | Your ROS packages: interfaces, executables, the install. | Automatically when the container is created, then by hand after an interface change. |

## Where the files are

```text
/workspace/              the repository, shared with the host
/opt/colcon_ws/build/    Colcon's working files, in the container
/opt/colcon_ws/install/  the built ROS packages, in the container
/opt/colcon_ws/log/      build logs, in the container
```

`/opt/colcon_ws` is not mounted on the host, so build and install output never
lands in the repository. It survives stopping and starting the container, but
**Rebuild Container** discards it - every rebuild starts clean.
