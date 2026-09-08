"""The RWS side of the system: a managed node that owns the robot session.

Ported from the intra project's intranodes_pkg/robot_controller_node.py, whose
hand-rolled state machine maps onto the ROS 2 lifecycle:

unconfigured
on_configure  (read parameters, log in to the robot)
on_activate   (start the timers, open up for goals)
on_cleanup    (log out, drop the session)

The parameter file is re-read on every configure, so a changed YAML takes
effect without restarting the process:

    ros2 lifecycle set /robot_controller deactivate
    ros2 lifecycle set /robot_controller cleanup
    ros2 lifecycle set /robot_controller configure
    ros2 lifecycle set /robot_controller activate
"""

import inspect
import json
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass
from math import radians

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import Node as LifecycleNode
from rclpy.lifecycle import State, TransitionCallbackReturn
from rclpy.parameter import Parameter
from rclpy.timer import Timer
from sensor_msgs.msg import JointState

from robot_control.constants import RobotControllerConstants as RCC
from robot_control.conversions import joints_to_dipc_jointtarget, pose_to_dipc_robtarget
from robot_control.rws.interface import RWSInterface
from robot_control.rws.provider import NO_STATUS
from robot_control_msgs.action import ExecuteJointArray, ExecutePoseArray
from robot_control_msgs.msg import RobotJoints
from robot_control_msgs.srv import RobotRequestSrv


@dataclass(frozen=True)
class ParamKeys:
    CONFIG_FILE = "config_file"
    IP_ADDRESS = "connection.ip_address"
    PORT = "connection.port"
    USERNAME = "connection.username"
    PASSWORD = "connection.password"
    SEND_KEEPALIVE = "utility.send_keepalive"
    KEEPALIVE_INTERVAL = "utility.keepalive_interval"
    SEND_JOINT_STATES = "utility.send_joint_states"
    JOINT_STATES_HZ = "utility.joint_states_hz"
    DIPC_RETRY_MAX = "dipc.retry_max"
    DIPC_RETRY_DELAY_S = "dipc.retry_delay_s"
    MOTION_START_TIMEOUT_S = "motion.start_timeout_s"
    MOTION_TIMEOUT_S = "motion.completion_timeout_s"
    MOTION_STALL_S = "motion.stall_timeout_s"
    MOTION_POLL_S = "motion.poll_interval_s"


# Defaults match config/robot_control.yaml; they only apply when the node is
# started without a parameter file.
PARAMETER_DECLARATIONS = [
    (
        ParamKeys.CONFIG_FILE,
        "",
        ParameterDescriptor(
            description="YAML to re-read on every configure. Empty disables the reload.",
            type=Parameter.Type.STRING.value,
        ),
    ),
    (
        ParamKeys.IP_ADDRESS,
        "192.168.0.31",
        ParameterDescriptor(
            description="Robot controller IP address", type=Parameter.Type.STRING.value
        ),
    ),
    (
        ParamKeys.PORT,
        80,
        ParameterDescriptor(
            description="Robot controller port (443 for real robot, 80 for simulation)",
            type=Parameter.Type.INTEGER.value,
        ),
    ),
    (
        ParamKeys.USERNAME,
        "Admin",
        ParameterDescriptor(
            description="Robot controller username for authentication",
            type=Parameter.Type.STRING.value,
        ),
    ),
    (
        ParamKeys.PASSWORD,
        "robotics",
        ParameterDescriptor(
            description="Robot controller password for authentication",
            type=Parameter.Type.STRING.value,
        ),
    ),
    (
        ParamKeys.SEND_KEEPALIVE,
        True,
        ParameterDescriptor(
            description="Enable keepalive messages to maintain connection",
            type=Parameter.Type.BOOL.value,
        ),
    ),
    (
        ParamKeys.KEEPALIVE_INTERVAL,
        120.0,
        ParameterDescriptor(
            description="Keepalive message interval in seconds",
            type=Parameter.Type.DOUBLE.value,
        ),
    ),
    (
        ParamKeys.SEND_JOINT_STATES,
        True,
        ParameterDescriptor(
            description="Enable joint states publishing", type=Parameter.Type.BOOL.value
        ),
    ),
    (
        ParamKeys.JOINT_STATES_HZ,
        8.0,
        ParameterDescriptor(
            description="Joint states publishing frequency in Hz",
            type=Parameter.Type.DOUBLE.value,
        ),
    ),
    (
        ParamKeys.DIPC_RETRY_MAX,
        100,
        ParameterDescriptor(
            description="Maximum number of retries when DIPC queue is full (status 500)",
            type=Parameter.Type.INTEGER.value,
        ),
    ),
    (
        ParamKeys.DIPC_RETRY_DELAY_S,
        0.25,
        ParameterDescriptor(
            description="Delay in seconds between DIPC retries",
            type=Parameter.Type.DOUBLE.value,
        ),
    ),
    (
        ParamKeys.MOTION_START_TIMEOUT_S,
        5.0,
        ParameterDescriptor(
            description="How long to wait for RAPID to confirm the routine started",
            type=Parameter.Type.DOUBLE.value,
        ),
    ),
    (
        ParamKeys.MOTION_TIMEOUT_S,
        300.0,
        ParameterDescriptor(
            description="How long a trajectory may take before the goal is aborted",
            type=Parameter.Type.DOUBLE.value,
        ),
    ),
    (
        ParamKeys.MOTION_STALL_S,
        30.0,
        ParameterDescriptor(
            description="Abort when the robot has not moved for this long mid-trajectory",
            type=Parameter.Type.DOUBLE.value,
        ),
    ),
    (
        ParamKeys.MOTION_POLL_S,
        0.5,
        ParameterDescriptor(
            description="How often the robot is asked whether it has finished",
            type=Parameter.Type.DOUBLE.value,
        ),
    ),
]

# Reading the robot state can fail on its own now and then; only a run of
# failures means the controller is really gone.
READ_FAILURES_MAX = 5

# Joint noise below this is the robot standing still, not moving (degrees).
JOINT_EPSILON_DEG = 0.01

# What the URDF calls the six axes. The controller answers with rax_1..rax_6.
JOINT_NAMES = [
    "Revolute 1",
    "Revolute 2",
    "Revolute 3",
    "Revolute 4",
    "Revolute 5",
    "Revolute 6",
]


def flatten_ros_parameters(
    values: dict[str, object], prefix: str = ""
) -> list[tuple[str, object]]:
    """Turn the nested YAML into the dotted keys the node declares."""
    flat: list[tuple[str, object]] = []
    for key, value in values.items():
        name = f"{prefix}{key}"
        if isinstance(value, dict):
            flat.extend(flatten_ros_parameters(value, f"{name}."))
        else:
            flat.append((name, value))
    return flat


class RobotControllerNode(LifecycleNode):
    def __init__(self):
        super().__init__(RCC.NODE_NAME)
        self.logger = self.get_logger()
        self.cb_group = ReentrantCallbackGroup()

        self.RWS: RWSInterface | None = None
        self._logged_in = False
        self.keepalive_timer: Timer | None = None
        self.joint_states_timer: Timer | None = None

        # rclpy exposes no supported accessor for the current lifecycle state,
        # so the goal callbacks read this instead.
        self._active = False

        # Stands in for the idle -> busy transition of the original state
        # machine: one trajectory on the robot at a time.
        self._busy_lock = threading.Lock()
        self._busy = False

        # Echoed back by RAPID so a confirmation cannot be mistaken for the
        # leftover of an earlier goal. RAPID nums hold integers up to 2**23.
        self._request_id = 0

        declared_params = self.declare_parameters(
            namespace="",  # Empty because params already have dot notation
            parameters=PARAMETER_DECLARATIONS,
        )
        self.logger.info(f"Declared {len(declared_params)} parameters")

        # Services
        self.controller_request_service = self.create_service(
            RobotRequestSrv,
            f"{RCC.NODE_NAME}/{RCC.ServiceNames.CONTROLLER_REQUEST}",
            self.controller_request_cb,
            callback_group=self.cb_group,
        )

        # Topics
        self.joint_states_publisher = self.create_lifecycle_publisher(
            JointState, f"{RCC.NODE_NAME}/{RCC.TopicNames.JOINT_STATES_TOPIC}", 10
        )

        # Actions

        self.robot_robtarget_move_action_server = ActionServer(
            self,
            ExecutePoseArray,
            f"{RCC.NODE_NAME}/{RCC.ActionNames.ROBOT_ROBTARGET_MOVE_ACTION}",
            execute_callback=self.execute_pose_array_cb,
            goal_callback=self.goal_callback_pose,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group,
        )

        self.robot_jointtarget_move_action_server = ActionServer(
            self,
            ExecuteJointArray,
            f"{RCC.NODE_NAME}/{RCC.ActionNames.ROBOT_JOINTTARGET_MOVE_ACTION}",
            execute_callback=self.execute_joint_array_cb,
            goal_callback=self.goal_callback_joint,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group,
        )

        self.logger.info(f"Boot of {RCC.NODE_NAME} complete, waiting for configuration")

    # ============= TYPED PARAMETER ACCESS =============
    # Parameter.value is untyped in rclpy, so every read is Unknown | None to
    # a checker. These narrow it to what ParameterDescriptor already declared

    def _param_str(self, key: str) -> str:
        value = self.get_parameter(key).value
        if not isinstance(value, str):
            raise TypeError(f"Parameter {key} is not a string: {value!r}")
        return value

    def _param_int(self, key: str) -> int:
        value = self.get_parameter(key).value
        if not isinstance(value, int):
            raise TypeError(f"Parameter {key} is not an int: {value!r}")
        return value

    def _param_float(self, key: str) -> float:
        value = self.get_parameter(key).value
        if not isinstance(value, (int, float)):
            raise TypeError(f"Parameter {key} is not a float: {value!r}")
        return float(value)

    def _param_bool(self, key: str) -> bool:
        value = self.get_parameter(key).value
        if not isinstance(value, bool):
            raise TypeError(f"Parameter {key} is not a bool: {value!r}")
        return value

    # ============= LIFECYCLE TRANSITIONS =============

    def _reload_parameters_from_file(self) -> None:
        """Re-read the parameter file so an edited YAML takes effect here.

        Launch loads it only at start-up. The file wins over `ros2 param set`.
        """
        path = self._param_str(ParamKeys.CONFIG_FILE)
        if not path:
            return

        with open(path) as handle:
            document = yaml.safe_load(handle) or {}

        # A parameter file addresses the node by name, or by the /** wildcard.
        section = document.get(self.get_name()) or document.get("/**") or {}
        entries = flatten_ros_parameters(section.get("ros__parameters", {}))

        declared = {name for name, _, _ in PARAMETER_DECLARATIONS}
        unknown = [name for name, _ in entries if name not in declared]
        if unknown:
            self.logger.warning(
                f"Ignoring keys in {path} that the node does not declare: "
                f"{', '.join(sorted(unknown))}"
            )

        updates = [
            Parameter(name, value=value) for name, value in entries if name in declared
        ]
        results = self.set_parameters(updates) if updates else []

        failed = [
            f"{u.name} ({r.reason})"
            for u, r in zip(updates, results, strict=True)
            if not r.successful
        ]
        if failed:
            self.logger.error(
                f"Rejected by {path}, kept old value: {', '.join(failed)}"
            )

        applied = len(updates) - len(failed)
        self.logger.info(f"Reloaded {applied} parameters from {path}")

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """Open the robot session. Every connection setting is read here."""
        if self.RWS is not None or self._logged_in:
            self.logger.error("Already configured, clean up before configuring again")
            return TransitionCallbackReturn.FAILURE

        try:
            self._reload_parameters_from_file()
        except Exception as e:
            self.logger.error(f"Could not read the parameter file: {e}")
            return TransitionCallbackReturn.FAILURE

        try:
            self.logger.info("Initializing RWSInterface...")
            rws = RWSInterface(
                host=self._param_str(ParamKeys.IP_ADDRESS),
                username=self._param_str(ParamKeys.USERNAME),
                password=self._param_str(ParamKeys.PASSWORD),
                port=self._param_int(ParamKeys.PORT),
                logger=self.logger,
            )
        except Exception as e:
            self.logger.error(f"Failed to initialize RWSInterface: {e}")
            return TransitionCallbackReturn.FAILURE

        try:
            if not rws.login():
                self.logger.error("Login failed, cannot configure")
                return TransitionCallbackReturn.FAILURE
        except Exception as e:
            self.logger.error(f"Login failed: {e}")
            return TransitionCallbackReturn.FAILURE

        self.RWS = rws
        self._logged_in = True
        self._seed_request_id(rws)
        self.logger.info("Login successful")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Start the timers and open up for motion goals."""
        if not self._logged_in:
            self.logger.error("Not logged in to the robot, cannot activate")
            return TransitionCallbackReturn.FAILURE

        if self._param_bool(ParamKeys.SEND_KEEPALIVE):
            period = self._param_float(ParamKeys.KEEPALIVE_INTERVAL)
            self.keepalive_timer = self.create_timer(
                period, self.keepalive_callback, callback_group=self.cb_group
            )

        if self._param_bool(ParamKeys.SEND_JOINT_STATES):
            period = self._param_float(ParamKeys.JOINT_STATES_HZ)
            period = (
                1.0 / period if period > 0 else 1.0
            )  # Convert Hz to seconds, default to 1 Hz if invalid
            self.joint_states_timer = self.create_timer(
                period, self.joint_states_callback, callback_group=self.cb_group
            )

        self._active = True
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Stop the timers and refuse new goals. The robot session stays open,
        so activating again does not have to log in a second time."""
        self._active = False
        self._stop_timers()
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self._release()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self._release()
        return TransitionCallbackReturn.SUCCESS

    def on_error(self, state: State) -> TransitionCallbackReturn:
        self._release()
        return TransitionCallbackReturn.SUCCESS

    def _stop_timers(self):
        if self.keepalive_timer is not None:
            self.keepalive_timer.destroy()
            self.keepalive_timer = None
        if self.joint_states_timer is not None:
            self.joint_states_timer.destroy()
            self.joint_states_timer = None

    def _release(self):
        """Drop the robot session. Deliberately never raises, so a cleanup
        always completes even when the controller is already gone."""
        self._active = False
        try:
            self._stop_timers()
            if self.RWS is not None:
                self.RWS.logout()
        except Exception as e:
            self.logger.warning(f"Error while releasing the robot session: {e}")
        finally:
            self.RWS = None
            self._logged_in = False
            # Claimed in one callback, dropped in another: a goal that never
            # reaches execute would shut the gate for good.
            self._busy = False

    # ============= TIMER CALLBACKS =============

    def keepalive_callback(self):
        if not self._logged_in:
            self.logger.info("Not logged in, skipping keepalive.")
            return

        self.logger.info("Sending keepalive signal...")
        try:
            if self.RWS is None:
                self.logger.error("Keepalive skipped: no robot session")
                self._logged_in = False
                return
            # send_keepalive reports the outcome itself.
            if not self.RWS.send_keepalive():
                self._logged_in = False
                self.logger.error("Keepalive failed, treating session as lost")

        except Exception as e:
            self._logged_in = False
            self.logger.error(f"Keepalive failed: {e}")

    def joint_states_callback(self):
        if self._logged_in:
            try:
                if self.RWS is None:
                    self.logger.error("Joint states skipped: no robot session")
                    return
                result_json, status_code = self.RWS.get_robot_joint_positions()
                if status_code != 200:
                    self.logger.warning(
                        f"Failed to get joint states (status={status_code}): {result_json}"
                    )
                    return

                data = json.loads(result_json)

                positions_deg = []
                for i in range(1, 7):
                    key = f"rax_{i}"
                    value = data.get(key, None)
                    if value is None:
                        raise ValueError(f"Missing joint value for {key}")
                    positions_deg.append(float(value))

                msg = JointState()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.name = JOINT_NAMES
                msg.position = [radians(v) for v in positions_deg]

                self.joint_states_publisher.publish(msg)

            except Exception as e:
                self.logger.error(f"Failed to get joint states: {e}")
        else:
            self.logger.info("Not logged in, skipping joint states retrieval.")

    # ============= GOAL ADMISSION =============

    def _claim(self) -> bool:
        """Take the robot for one goal. False means another goal already has it.

        Covers only the sending phase; the motion is guarded by the is_rapid_idle
        check in the goal callbacks - RAPID holds CURRENT_STATE non-zero until done.
        """
        with self._busy_lock:
            if self._busy:
                return False
            self._busy = True
            return True

    def _unclaim(self):
        with self._busy_lock:
            self._busy = False

    def cancel_callback(self, goal_handle):
        """Accept cancel requests."""
        self.logger.info("Cancel requested for DIPC trajectory execution")
        return CancelResponse.ACCEPT

    def goal_callback_pose(self, goal_request: ExecutePoseArray.Goal) -> GoalResponse:
        self.logger.info(
            f"Received new goal with {len(goal_request.path.poses)} poses for DIPC trajectory execution"
        )

        if not self._active:
            self.logger.error("Goal rejected: node is not active")
            return GoalResponse.REJECT

        if len(goal_request.path.poses) == 0:
            self.logger.error("Goal rejected: empty PoseArray")
            return GoalResponse.REJECT

        if not self._claim():
            self.logger.error("Goal rejected: the robot is already executing a goal")
            return GoalResponse.REJECT

        try:
            if self.RWS is None:
                self.logger.error("Goal rejected: robot state client is unavailable")
                self._unclaim()
                return GoalResponse.REJECT

            if not self.RWS.is_rapid_idle():
                self.logger.error("Goal rejected: Robot is not in idle state")
                self._unclaim()
                return GoalResponse.REJECT
        except Exception as e:
            self.logger.error(f"Goal rejected: could not read the robot state: {e}")
            self._unclaim()
            return GoalResponse.REJECT

        self.logger.info(
            f"Goal accepted: {len(goal_request.path.poses)} poses to send via DIPC"
        )
        return GoalResponse.ACCEPT

    def goal_callback_joint(self, goal_request: ExecuteJointArray.Goal) -> GoalResponse:
        waypoints: list[RobotJoints] = list(goal_request.waypoints)
        self.logger.info(
            f"Received joint trajectory goal with {len(waypoints)} waypoints"
        )

        if not self._active:
            self.logger.error("Goal rejected: node is not active")
            return GoalResponse.REJECT

        if len(waypoints) == 0:
            self.logger.error("Goal rejected: empty joint trajectory")
            return GoalResponse.REJECT

        if not self._claim():
            self.logger.error("Goal rejected: the robot is already executing a goal")
            return GoalResponse.REJECT

        try:
            if self.RWS is None:
                self.logger.error("Goal rejected: robot state client is unavailable")
                self._unclaim()
                return GoalResponse.REJECT

            if not self.RWS.is_rapid_idle():
                self.logger.error("Goal rejected: robot not idle")
                self._unclaim()
                return GoalResponse.REJECT
        except Exception as e:
            self.logger.error(f"Goal rejected: could not read the robot state: {e}")
            self._unclaim()
            return GoalResponse.REJECT

        self.logger.info(f"Joint trajectory goal accepted: {len(waypoints)} waypoints")
        return GoalResponse.ACCEPT

    # ============= ACTION SERVER CALLBACKS =============

    def _seed_request_id(self, rws: RWSInterface) -> None:
        """Start the ids above whatever the controller still remembers.

        RAPID keeps the last echo across restarts of this node, so a counter
        that begins at zero would match an old confirmation.
        """
        seen = [
            self._read_num(rws, symbol, RCC.Modules.USER)
            for symbol in (
                RCC.Symbols.ACCEPTED_ID,
                RCC.Symbols.COMPLETED_ID,
                RCC.Symbols.REJECTED_ID,
            )
        ]
        known = [value for value in seen if value is not None]
        if not known:
            self.logger.warning(
                "Could not read the RAPID handshake ids, seeding by clock"
            )
            self._request_id = int(time.time()) % 999999
            return

        self._request_id = max(known)
        self.logger.info(f"Handshake ids continue from {self._request_id}")

    def _next_request_id(self) -> int:
        """A fresh id for the next goal. Never 0 - that is RAPID's "none"."""
        self._request_id = self._request_id % 999999 + 1
        return self._request_id

    def _read_num(self, rws: RWSInterface, symbol: str, module: str) -> int | None:
        """A RAPID num as an int, or None when the controller would not say."""
        value, status = rws.get_rapid_symbol(symbol, module)
        if status != 200:
            return None
        try:
            return int(float(value))
        except ValueError:
            return None

    def _start_routine(self, rws: RWSInterface, routine_name: str, speed) -> int:
        """Set a routine running and wait for RAPID to confirm it entered.

        Writing the state only asks; an unknown routine name leaves the state
        machine back at idle within milliseconds and nothing else happens.
        """
        request_id = self._next_request_id()
        writes = [
            (str(request_id), RCC.Symbols.REQUEST_ID, RCC.Modules.USER, 0.1),
            (f'"{routine_name}"', RCC.Symbols.ROUTINE_NAME, RCC.Modules.RAPID, 0.1),
            (speed, RCC.Symbols.SPEED, RCC.Modules.USER, 0.1),
            (RCC.States.EXECUTE, RCC.Symbols.CURRENT_STATE, RCC.Modules.MAIN, 0.0),
        ]

        for value, symbol, module, settle_s in writes:
            message, status = rws.set_rapid_symbol_raw(value, symbol, module)
            if status != 204:
                raise RuntimeError(
                    f"Could not set RAPID symbol {symbol} in {module}: "
                    f"{message} (status={status})"
                )
            # The controller needs a moment before the next write lands.
            time.sleep(settle_s)

        self._await_routine_start(
            rws,
            request_id,
            routine_name,
            self._param_float(ParamKeys.MOTION_START_TIMEOUT_S),
            self._param_float(ParamKeys.MOTION_POLL_S),
        )
        return request_id

    def _await_routine_start(
        self,
        rws: RWSInterface,
        request_id: int,
        routine_name: str,
        timeout_s: float,
        poll_s: float,
    ) -> None:
        """Block until the routine says it is in and has emptied the queue.

        Sending points before that confirmation risks RMQEmptyQueue swallowing
        them, so this is also what makes the first send safe.
        """
        deadline = time.monotonic() + timeout_s

        while time.monotonic() < deadline:
            if self._read_num(rws, RCC.Symbols.ACCEPTED_ID, RCC.Modules.USER) == (
                request_id
            ):
                self.logger.info(f"RAPID confirmed '{routine_name}' is running")
                return
            if self._read_num(rws, RCC.Symbols.REJECTED_ID, RCC.Modules.USER) == (
                request_id
            ):
                raise RuntimeError(
                    f"The controller does not know a routine called '{routine_name}'"
                )
            time.sleep(poll_s)

        raise RuntimeError(
            f"RAPID never confirmed '{routine_name}' started within {timeout_s:.0f} s"
        )

    def _end_buffer_routine(
        self,
        rws: RWSInterface,
        message: str | None,
        retry_max: int,
        retry_delay_s: float,
    ) -> None:
        """Let the RAPID buffer routine finish after a goal fell over.

        Its loop only ends on a userdef=2 message, so dropping out without one
        leaves the robot busy for good and every later goal gets rejected.
        """
        if message is None:
            self.logger.error("Nothing was queued, RAPID stays busy until restarted")
            return

        for _ in range(retry_max + 1):
            _, status = rws.send_dipc_message(message=message, userdef="2")
            if status == 204:
                self.logger.info("Buffer routine released, robot stops after the queue")
                return
            time.sleep(retry_delay_s)

        self.logger.error("Could not release the buffer routine, RAPID stays busy")

    def _read_joints(self, rws: RWSInterface) -> list[float] | None:
        """The six joint angles, or None when the controller would not say."""
        message, status = rws.get_robot_joint_positions()
        if status != 200:
            return None
        data = json.loads(message)
        return [float(data[f"rax_{i}"]) for i in range(1, 7)]

    def _wait_for_motion_end(
        self,
        rws: RWSInterface,
        request_id: int,
        on_progress: Callable[[int], None],
        timeout_s: float,
        stall_s: float,
        poll_s: float,
    ) -> tuple[bool, str, int]:
        """Block until the routine reports this goal finished.

        Queueing a point is not executing it, and the state machine going idle
        is not proof either - only the echoed id says the robot arrived.
        """
        started = time.monotonic()
        last_progress = started
        last_joints: list[float] | None = None
        moves_done = 0
        read_failures = 0

        def finished_now() -> tuple[bool, str, int] | None:
            """The routine's own confirmation, once it has been written."""
            if self._read_num(rws, RCC.Symbols.COMPLETED_ID, RCC.Modules.USER) != (
                request_id
            ):
                return None
            done = self._read_num(rws, RCC.Symbols.MOVES_DONE, RCC.Modules.USER)
            count = moves_done if done is None else done
            on_progress(count)
            elapsed = time.monotonic() - started
            return True, f"Robot finished in {elapsed:.1f} s", count

        while True:
            if not self._active:
                return (
                    False,
                    "Node was deactivated while the robot was still moving",
                    moves_done,
                )
            if not self._logged_in:
                return (
                    False,
                    "Robot session was lost while the robot was still moving",
                    moves_done,
                )

            joints = None
            try:
                outcome = finished_now()
                if outcome is not None:
                    return outcome

                # Idle and stopped both leave the robot motionless, and neither
                # of them means this goal is done.
                if not rws.is_running():
                    return (
                        False,
                        "RAPID stopped before the trajectory finished",
                        moves_done,
                    )

                state, status = rws.get_rapid_symbol(
                    RCC.Symbols.CURRENT_STATE, RCC.Modules.MAIN
                )
                if status != 200:
                    raise RuntimeError(f"reading the robot state answered {status}")
                if state.strip() == RCC.States.IDLE:
                    # The routine writes completed_id a hair before the state
                    # machine drops to idle, so look once more before failing.
                    outcome = finished_now()
                    if outcome is not None:
                        return outcome
                    return (
                        False,
                        ("The RAPID routine ended without finishing the trajectory"),
                        moves_done,
                    )

                done = self._read_num(rws, RCC.Symbols.MOVES_DONE, RCC.Modules.USER)
                if done is not None and done != moves_done:
                    moves_done = done
                    on_progress(moves_done)
                    last_progress = time.monotonic()

                joints = self._read_joints(rws)
                read_failures = 0
            except Exception as e:
                read_failures += 1
                if read_failures >= READ_FAILURES_MAX:
                    return False, f"Lost contact with the controller: {e}", moves_done
                self.logger.warning(f"Could not read the robot state: {e}")

            now = time.monotonic()
            if joints is not None and (
                last_joints is None
                or any(
                    abs(new - old) > JOINT_EPSILON_DEG
                    for new, old in zip(joints, last_joints)
                )
            ):
                last_joints = joints
                last_progress = now

            if now - last_progress > stall_s:
                return (
                    False,
                    (
                        f"Robot has not moved for {stall_s:.0f} s but RAPID is still busy"
                    ),
                    moves_done,
                )
            if now - started > timeout_s:
                return (
                    False,
                    f"Trajectory did not finish within {timeout_s:.0f} s",
                    moves_done,
                )

            time.sleep(poll_s)

    def execute_pose_array_cb(
        self, goal_handle: ServerGoalHandle
    ) -> ExecutePoseArray.Result:
        """
        Execute callback for the DIPC trajectory action.
        """
        try:
            return self._execute_pose_array(goal_handle)
        finally:
            self._unclaim()

    def _execute_pose_array(
        self, goal_handle: ServerGoalHandle
    ) -> ExecutePoseArray.Result:
        goal: ExecutePoseArray.Goal = goal_handle.request

        try:
            # Inside the try: a throw out here would leave the goal unterminated.
            if self.RWS is None:
                raise RuntimeError("No robot session, configure the node first")
            rws = self.RWS

            dipc_retry_max = self._param_int(ParamKeys.DIPC_RETRY_MAX)
            dipc_retry_delay_s = self._param_float(ParamKeys.DIPC_RETRY_DELAY_S)

            MC = RCC.MotionCommands
            routines = {MC.MOVE_L: RCC.Routines.MOVE_L, MC.MOVE_J: RCC.Routines.MOVE_J}
            if goal.motion_command not in routines:
                raise ValueError(f"Unsupported motion command: {goal.motion_command}")

            request_id = self._start_routine(
                rws, routines[goal.motion_command], goal.speed
            )

        except Exception as e:
            error_msg = f"Error in execute_pose_array_cb: {e}"
            self.logger.error(error_msg)
            goal_handle.abort()
            result = ExecutePoseArray.Result()
            result.success = False
            result.message = error_msg
            result.executed_count = 0
            return result

        feedback_msg = ExecutePoseArray.Feedback()
        result = ExecutePoseArray.Result()

        poses = goal_handle.request.path.poses
        total = len(poses)
        sent_count = 0
        curr_pose = 0
        # Kept for the abort paths below, which have to close the RAPID loop.
        last_message: str | None = None
        # A userdef=2 message the robot took. Without one the RAPID loop never
        # ends, so every way out of here has to leave one behind.
        terminator_sent = False
        # Set when sending ends early. Either way the robot still has to work
        # through what is already queued, so nothing returns before the wait.
        send_error: str | None = None
        cancelled = False

        self.logger.info(f"Starting DIPC trajectory execution with {total} poses")

        try:
            while curr_pose < total:
                pose = poses[curr_pose]

                # Determine userdef: 2 for last point, 1 otherwise.
                is_last = curr_pose == total - 1
                userdef = "2" if is_last else "1"

                # If a cancel has been requested, promote this point to the last point
                if goal_handle.is_cancel_requested:
                    userdef = "2"

                robtarget_str = pose_to_dipc_robtarget(pose)
                last_message = robtarget_str

                retries = 0
                while True:
                    # If cancel arrives during retries
                    if goal_handle.is_cancel_requested and userdef != "2":
                        self.logger.info(
                            f"Cancel requested during retries at pose {curr_pose}/{total}"
                        )
                        userdef = "2"

                    _, status_code = rws.send_dipc_message(
                        message=robtarget_str, userdef=userdef
                    )

                    if status_code == 204:
                        if userdef == "2":
                            terminator_sent = True
                        break

                    if status_code == 500 and retries < dipc_retry_max:
                        retries += 1
                        time.sleep(dipc_retry_delay_s)
                        continue

                    send_error = (
                        f"DIPC send failed at pose {curr_pose + 1}/{total}: "
                        f"(status={status_code}, retries={retries})"
                    )
                    self.logger.error(send_error)
                    if not terminator_sent:
                        self._end_buffer_routine(
                            rws, last_message, dipc_retry_max, dipc_retry_delay_s
                        )
                    break

                # That break only left the retry loop; the queue is closed now.
                if send_error is not None:
                    break

                sent_count += 1

                feedback_msg.current_index = curr_pose
                feedback_msg.state = f"Sent pose {curr_pose + 1}/{total}"
                pose_stamped = PoseStamped()
                pose_stamped.header.stamp = self.get_clock().now().to_msg()
                pose_stamped.pose = pose
                feedback_msg.current_pose = pose_stamped
                goal_handle.publish_feedback(feedback_msg)

                curr_pose += 1

                # If cancel was requested
                if goal_handle.is_cancel_requested:
                    # The cancel can land after the send, with this point already
                    # queued as a fly-by. Repeating it as the last one ends the loop.
                    if not terminator_sent:
                        self._end_buffer_routine(
                            rws, last_message, dipc_retry_max, dipc_retry_delay_s
                        )
                    cancelled = True
                    break

            self.logger.info(f"All {total} poses queued, waiting for the robot")

        except Exception as e:
            send_error = f"Error during DIPC trajectory execution: {e}"
            self.logger.error(send_error)
            if not terminator_sent:
                self._end_buffer_routine(
                    rws, last_message, dipc_retry_max, dipc_retry_delay_s
                )

        def report_progress(done: int) -> None:
            feedback_msg.current_index = max(done - 1, 0)
            feedback_msg.state = f"Executed {done}/{total}"
            goal_handle.publish_feedback(feedback_msg)

        # However sending ended, the robot still has to work through the queue.
        finished, wait_message, executed = self._wait_for_motion_end(
            rws,
            request_id,
            report_progress,
            self._param_float(ParamKeys.MOTION_TIMEOUT_S),
            self._param_float(ParamKeys.MOTION_STALL_S),
            self._param_float(ParamKeys.MOTION_POLL_S),
        )

        result.executed_count = executed
        if not finished:
            result.success = False
            result.message = wait_message
            self.logger.error(wait_message)
            goal_handle.abort()
        elif send_error is not None:
            result.success = False
            result.message = send_error
            goal_handle.abort()
        elif cancelled or goal_handle.is_cancel_requested:
            # A cancel during the wait cannot unqueue anything, but the caller
            # still asked for one and the goal has to end as cancelled.
            result.success = False
            result.message = f"Cancelled after {executed} of {total} points"
            self.logger.info(result.message)
            goal_handle.canceled()
        else:
            result.success = True
            result.message = wait_message
            self.logger.info(wait_message)
            goal_handle.succeed()
        return result

    def execute_joint_array_cb(
        self, goal_handle: ServerGoalHandle
    ) -> ExecuteJointArray.Result:
        """
        Execute callback for the joint trajectory DIPC action.
        """
        try:
            return self._execute_joint_array(goal_handle)
        finally:
            self._unclaim()

    def _execute_joint_array(
        self, goal_handle: ServerGoalHandle
    ) -> ExecuteJointArray.Result:
        goal: ExecuteJointArray.Goal = goal_handle.request

        try:
            # Inside the try, for the same reason as in _execute_pose_array.
            if self.RWS is None:
                raise RuntimeError("No robot session, configure the node first")
            rws = self.RWS

            dipc_retry_max = self._param_int(ParamKeys.DIPC_RETRY_MAX)
            dipc_retry_delay_s = self._param_float(ParamKeys.DIPC_RETRY_DELAY_S)

            waypoints: list[RobotJoints] = list(goal.waypoints)

            routines = {
                RCC.MotionCommands.MOVE_ABS_J: RCC.Routines.MOVE_ABS_J,
                RCC.MotionCommands.MOVE_ABS_L: RCC.Routines.MOVE_ABS_L,
            }
            if goal.motion_command not in routines:
                raise ValueError(f"Unsupported motion command: {goal.motion_command}")

            request_id = self._start_routine(
                rws, routines[goal.motion_command], goal.speed
            )

        except Exception as e:
            error_msg = f"Error in execute_joint_array_cb setup: {e}"
            self.logger.error(error_msg)
            goal_handle.abort()
            result = ExecuteJointArray.Result()
            result.success = False
            result.message = error_msg
            result.executed_count = 0
            return result

        feedback_msg = ExecuteJointArray.Feedback()
        result = ExecuteJointArray.Result()

        total = len(waypoints)
        sent_count = 0
        curr_idx = 0
        # Kept for the abort paths below, which have to close the RAPID loop.
        last_message: str | None = None
        # A userdef=2 message the robot took. Without one the RAPID loop never
        # ends, so every way out of here has to leave one behind.
        terminator_sent = False
        # Set when sending ends early. Either way the robot still has to work
        # through what is already queued, so nothing returns before the wait.
        send_error: str | None = None
        cancelled = False

        self.logger.info(f"Starting DIPC joint trajectory with {total} waypoints")

        try:
            while curr_idx < total:
                wp: RobotJoints = waypoints[curr_idx]
                joints = [wp.j1, wp.j2, wp.j3, wp.j4, wp.j5, wp.j6]

                is_last = curr_idx == total - 1
                userdef = "2" if is_last else "1"

                if goal_handle.is_cancel_requested:
                    userdef = "2"

                jointtarget_str = joints_to_dipc_jointtarget(joints)
                last_message = jointtarget_str

                retries = 0
                while True:
                    if goal_handle.is_cancel_requested and userdef != "2":
                        self.logger.info(
                            f"Cancel requested during retries at waypoint {curr_idx}/{total}"
                        )
                        userdef = "2"

                    _, status_code = rws.send_dipc_message(
                        message=jointtarget_str, userdef=userdef
                    )

                    if status_code == 204:
                        if userdef == "2":
                            terminator_sent = True
                        break

                    if status_code == 500 and retries < dipc_retry_max:
                        retries += 1
                        time.sleep(dipc_retry_delay_s)
                        continue

                    send_error = (
                        f"DIPC send failed at waypoint {curr_idx + 1}/{total}: "
                        f"(status={status_code}, retries={retries})"
                    )
                    self.logger.error(send_error)
                    if not terminator_sent:
                        self._end_buffer_routine(
                            rws, last_message, dipc_retry_max, dipc_retry_delay_s
                        )
                    break

                # That break only left the retry loop; the queue is closed now.
                if send_error is not None:
                    break

                sent_count += 1

                feedback_msg.current_index = curr_idx
                feedback_msg.state = f"Sent waypoint {curr_idx + 1}/{total}"
                goal_handle.publish_feedback(feedback_msg)

                curr_idx += 1

                if goal_handle.is_cancel_requested:
                    # The cancel can land after the send, with this point already
                    # queued as a fly-by. Repeating it as the last one ends the loop.
                    if not terminator_sent:
                        self._end_buffer_routine(
                            rws, last_message, dipc_retry_max, dipc_retry_delay_s
                        )
                    cancelled = True
                    break

            self.logger.info(f"All {total} waypoints queued, waiting for the robot")

        except Exception as e:
            send_error = f"Error during joint trajectory execution: {e}"
            self.logger.error(send_error)
            if not terminator_sent:
                self._end_buffer_routine(
                    rws, last_message, dipc_retry_max, dipc_retry_delay_s
                )

        def report_progress(done: int) -> None:
            feedback_msg.current_index = max(done - 1, 0)
            feedback_msg.state = f"Executed {done}/{total}"
            goal_handle.publish_feedback(feedback_msg)

        # However sending ended, the robot still has to work through the queue.
        finished, wait_message, executed = self._wait_for_motion_end(
            rws,
            request_id,
            report_progress,
            self._param_float(ParamKeys.MOTION_TIMEOUT_S),
            self._param_float(ParamKeys.MOTION_STALL_S),
            self._param_float(ParamKeys.MOTION_POLL_S),
        )

        result.executed_count = executed
        if not finished:
            result.success = False
            result.message = wait_message
            self.logger.error(wait_message)
            goal_handle.abort()
        elif send_error is not None:
            result.success = False
            result.message = send_error
            goal_handle.abort()
        elif cancelled or goal_handle.is_cancel_requested:
            # A cancel during the wait cannot unqueue anything, but the caller
            # still asked for one and the goal has to end as cancelled.
            result.success = False
            result.message = f"Cancelled after {executed} of {total} points"
            self.logger.info(result.message)
            goal_handle.canceled()
        else:
            result.success = True
            result.message = wait_message
            self.logger.info(wait_message)
            goal_handle.succeed()
        return result

    # ============= REQUEST SERVICE =============

    def controller_request_cb(
        self, request: RobotRequestSrv.Request, response: RobotRequestSrv.Response
    ):
        """Handle GET request service call"""
        params: list[str] = [param for param in request.params]
        # filter out falsy params
        params = [p for p in params if p]
        cmd: str = request.command

        self.logger.info(
            f"Received controller_request: command='{cmd}' params={params}"
        )

        try:
            result = self.robot_request(cmd, params)
            response.message, response.status_code = result
            # An RWSResult knows whether the operation worked; a plain tuple
            # carries nothing but the HTTP status of its last request.
            outcome = getattr(result, "ok", None)
            response.status = (
                outcome if outcome is not None else 200 <= response.status_code < 300
            )

        except Exception as e:
            error_msg = f"Error handling controller_request '{cmd}': {e}"
            self.logger.error(error_msg)
            response.message = error_msg
            response.status_code = NO_STATUS
            response.status = False

        self.logger.info(
            f"Controller request response: status={response.status}  message='{response.message}'  status_code={response.status_code}"
        )

        return response

    def robot_request(self, cmd: str, params: list):

        if self._logged_in and self.RWS is not None:
            if not hasattr(self.RWS, cmd):
                raise ValueError(f"Method {cmd} not found")

            method = getattr(self.RWS, cmd)
            signature = inspect.signature(method)
            # Filter out 'self' parameter
            method_params = [
                p for name, p in signature.parameters.items() if name != "self"
            ]
            num_total_params = len(method_params)
            # Get number of required parameters
            num_required_params = sum(
                1 for p in method_params if p.default == inspect.Parameter.empty
            )

            try:
                if num_required_params == 0 and len(params) == 0:
                    # No parameters expected
                    return method()
                elif num_required_params == len(params):
                    # Exact match
                    return method(*params)
                elif (
                    len(params) <= num_total_params
                    and len(params) >= num_required_params
                ):
                    # Some optional parameters can be omitted
                    return method(*params)

                elif len(params) < num_required_params:
                    # Fewer parameters provided than required
                    raise ValueError(
                        f"Not enough parameters provided: got {len(params)}, need at least {num_required_params}"
                    )
                else:
                    # Too many parameters, use only what's needed
                    return method(*params[:num_total_params])

            except TypeError as e:
                raise ValueError(f"Parameter mismatch calling {cmd}: {e}")
        else:
            raise RuntimeError(
                "Node not initialized or not logged in to robot, reinitialize the node."
            )


def main(args=None):

    rclpy.init(args=args)
    robot_controller_node = None
    executor = None

    try:
        robot_controller_node = RobotControllerNode()

        # Create multithreaded executor with 3 threads
        executor = MultiThreadedExecutor(num_threads=3)
        executor.add_node(robot_controller_node)

        executor.spin()

    except KeyboardInterrupt:
        if robot_controller_node:
            robot_controller_node.logger.info("Keyboard interrupt received")

    except Exception as e:
        if robot_controller_node:
            robot_controller_node.logger.error(f"Unexpected error: {e}")
        else:
            print(f"Error during node initialization: {e}")

    finally:
        # Shutdown executor (stops spinning and waits for callbacks to finish)
        if executor:
            executor.shutdown(timeout_sec=5)

        # Let go of the robot even if the lifecycle never got a shutdown
        if robot_controller_node:
            robot_controller_node._release()
            robot_controller_node.destroy_node()

        try:
            rclpy.shutdown()
        except Exception:  # noqa: S110
            pass  # rclpy may already be shut down


if __name__ == "__main__":
    main()
