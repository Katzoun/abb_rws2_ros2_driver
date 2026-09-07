"""The RWS side of the system: a managed node that owns the robot session.

Ported from the intra project's intranodes_pkg/robot_controller_node.py. That
node ran a hand-rolled state machine (core_pkg/nodes_core.py) over a plain
rclpy Node; here the same states are expressed as a real ROS 2 lifecycle:

    not_initialized   ->  unconfigured
    initialize_node   ->  on_configure  (read parameters, log in to the robot)
                      +   on_activate   (start the timers, open up for goals)
    cleanup_resources ->  on_cleanup    (log out, drop the session)

Configuration is reloaded by taking the node back around that loop:

    ros2 lifecycle set /robot_controller deactivate
    ros2 lifecycle set /robot_controller cleanup
    ros2 param set /robot_controller connection.ip_address 192.168.0.37
    ros2 lifecycle set /robot_controller configure
    ros2 lifecycle set /robot_controller activate

The original had a busy state that doubled as an admission gate: a goal was
accepted only if the idle -> busy transition was legal, which kept two
trajectories from ever overlapping. The lifecycle has no such state, so that
job is done here by an explicit claim (see _claim).
"""

import inspect
import json
import threading
import time
from dataclasses import dataclass
from math import radians

import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import Node as LifecycleNode, State, TransitionCallbackReturn
from rclpy.parameter import Parameter

from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

from robot_control_msgs.action import ExecutePoseArray, ExecuteJointArray
from robot_control_msgs.msg import RobotJoints
from robot_control_msgs.srv import RobotRequestSrv

from robot_control.constants import RobotControllerConstants as RCC
from robot_control.rws.interface import RWSInterface


@dataclass(frozen=True)
class ParamKeys:
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


# The defaults match config/robot_control.yaml, which is what the launch file
# loads. They only come into play when the node is started bare, without a
# parameter file.
PARAMETER_DECLARATIONS = [
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
]

# What the URDF calls the six axes. The controller answers with rax_1..rax_6.
JOINT_NAMES = [
    "Revolute 1",
    "Revolute 2",
    "Revolute 3",
    "Revolute 4",
    "Revolute 5",
    "Revolute 6",
]


class RobotControllerNode(LifecycleNode):
    def __init__(self):
        super().__init__(RCC.NODE_NAME)
        self.logger = self.get_logger()
        self.cb_group = ReentrantCallbackGroup()

        self.RWS = None
        self._logged_in = False
        self.keepalive_timer = None
        self.joint_states_timer = None

        # rclpy exposes no supported accessor for the current lifecycle state,
        # so the goal callbacks read this instead.
        self._active = False

        # Stands in for the idle -> busy transition of the original state
        # machine: one trajectory on the robot at a time.
        self._busy_lock = threading.Lock()
        self._busy = False

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

    # ============= LIFECYCLE TRANSITIONS =============

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """Open the robot session. Everything the connection needs is read here,
        which is why changing a parameter means cleaning up and configuring
        again for it to take effect."""
        if self.RWS is not None or self._logged_in:
            self.logger.error("Already configured, clean up before configuring again")
            return TransitionCallbackReturn.FAILURE

        try:
            self.logger.info("Initializing RWSInterface...")
            self.RWS = RWSInterface(
                host=self.get_parameter(ParamKeys.IP_ADDRESS).value,
                username=self.get_parameter(ParamKeys.USERNAME).value,
                password=self.get_parameter(ParamKeys.PASSWORD).value,
                port=self.get_parameter(ParamKeys.PORT).value,
                logger=self.logger,
            )
        except Exception as e:
            self.logger.error(f"Failed to initialize RWSInterface: {e}")
            self.RWS = None
            return TransitionCallbackReturn.FAILURE

        try:
            if not self.RWS.login():
                self.logger.error("Login failed: the controller refused the session")
                self.RWS = None
                return TransitionCallbackReturn.FAILURE
        except Exception as e:
            self.logger.error(f"Login failed: {e}")
            self.RWS = None
            return TransitionCallbackReturn.FAILURE

        self._logged_in = True
        self.logger.info("Login successful")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Start the timers and open up for motion goals."""
        if not self._logged_in:
            self.logger.error("Not logged in to the robot, cannot activate")
            return TransitionCallbackReturn.FAILURE

        if self.get_parameter(ParamKeys.SEND_KEEPALIVE).value:
            period = self.get_parameter(ParamKeys.KEEPALIVE_INTERVAL).value
            self.keepalive_timer = self.create_timer(
                period, self.keepalive_callback, callback_group=self.cb_group
            )

        if self.get_parameter(ParamKeys.SEND_JOINT_STATES).value:
            period = self.get_parameter(ParamKeys.JOINT_STATES_HZ).value
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

    # ============= TIMER CALLBACKS =============

    def keepalive_callback(self):
        if self._logged_in:
            self.logger.info("Sending keepalive signal...")
            self._logged_in = False
            try:
                result = self.RWS.send_keepalive()
                if result == True:
                    self._logged_in = True
                    self.logger.info("Keepalive successful")
                else:
                    self.logger.error("Keepalive failed: No response from robot")

            except Exception as e:
                self.logger.error(f"Keepalive failed: {e}")
        else:
            self.logger.info("Not logged in, skipping keepalive.")

    def joint_states_callback(self):
        if self._logged_in:
            try:
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
        """Take the robot for one goal. False means another goal already has it."""
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
        waypoints: list[RobotJoints] = goal_request.waypoints
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

        dipc_retry_max = self.get_parameter(ParamKeys.DIPC_RETRY_MAX).value
        dipc_retry_delay_s = self.get_parameter(ParamKeys.DIPC_RETRY_DELAY_S).value

        try:
            MC = RCC.MotionCommands
            if goal.motion_command not in [MC.MOVE_L, MC.MOVE_J]:
                raise ValueError(f"Unsupported motion command: {goal.motion_command}")

            if goal.motion_command == MC.MOVE_L:
                routine_name = RCC.Routines.MOVE_L
            elif goal.motion_command == MC.MOVE_J:
                routine_name = RCC.Routines.MOVE_J

            self.RWS.set_rapid_symbol_raw(
                f'"{routine_name}"', RCC.Symbols.ROUTINE_NAME, RCC.Modules.RAPID
            )
            time.sleep(0.1)
            self.RWS.set_rapid_symbol_raw(
                goal.speed, RCC.Symbols.SPEED, RCC.Modules.USER
            )
            time.sleep(0.1)
            self.RWS.set_rapid_symbol_raw(
                RCC.States.EXECUTE, RCC.Symbols.CURRENT_STATE, RCC.Modules.MAIN
            )
            time.sleep(0.3)

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

                robtarget_str = RWSInterface.pose_to_dipc_robtarget(pose)

                retries = 0
                while True:
                    # If cancel arrives during retries
                    if goal_handle.is_cancel_requested and userdef != "2":
                        self.logger.info(
                            f"Cancel requested during retries at pose {curr_pose}/{total}"
                        )
                        userdef = "2"

                    msg_result, status_code = self.RWS.send_dipc_message(
                        message=robtarget_str, userdef=userdef
                    )

                    if status_code == 204:
                        break

                    if status_code == 500 and retries < dipc_retry_max:
                        retries += 1
                        time.sleep(dipc_retry_delay_s)
                        continue

                    error_msg = (
                        f"DIPC send failed at pose {curr_pose + 1}/{total}: "
                        f"(status={status_code}, retries={retries})"
                    )
                    self.logger.error(error_msg)
                    goal_handle.abort()
                    result.success = False
                    result.message = error_msg
                    result.executed_count = sent_count
                    return result

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
                    goal_handle.canceled()
                    result.success = False
                    result.message = f"Cancelled after sending {sent_count} pose(s)"
                    result.executed_count = sent_count
                    self.logger.info(result.message)
                    return result

            # All poses sent successfully
            goal_handle.succeed()
            result.success = True
            result.message = f"All {total} poses sent successfully"
            result.executed_count = sent_count

            self.logger.info(result.message)
            return result

        except Exception as e:
            error_msg = f"Error during DIPC trajectory execution: {e}"
            self.logger.error(error_msg)
            goal_handle.abort()
            result.success = False
            result.message = error_msg
            result.executed_count = sent_count

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

        dipc_retry_max = self.get_parameter(ParamKeys.DIPC_RETRY_MAX).value
        dipc_retry_delay_s = self.get_parameter(ParamKeys.DIPC_RETRY_DELAY_S).value

        try:
            waypoints: list[RobotJoints] = goal.waypoints

            if goal.motion_command not in [
                RCC.MotionCommands.MOVE_ABS_J,
                RCC.MotionCommands.MOVE_ABS_L,
            ]:
                raise ValueError(f"Unsupported motion command: {goal.motion_command}")

            if goal.motion_command == RCC.MotionCommands.MOVE_ABS_J:
                routine_name = RCC.Routines.MOVE_ABS_J
            elif goal.motion_command == RCC.MotionCommands.MOVE_ABS_L:
                routine_name = RCC.Routines.MOVE_ABS_L

            self.RWS.set_rapid_symbol_raw(
                f'"{routine_name}"', RCC.Symbols.ROUTINE_NAME, RCC.Modules.RAPID
            )
            time.sleep(0.1)
            self.RWS.set_rapid_symbol_raw(
                goal.speed, RCC.Symbols.SPEED, RCC.Modules.USER
            )
            time.sleep(0.1)
            self.RWS.set_rapid_symbol_raw(
                RCC.States.EXECUTE, RCC.Symbols.CURRENT_STATE, RCC.Modules.MAIN
            )
            time.sleep(0.3)

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

        self.logger.info(f"Starting DIPC joint trajectory with {total} waypoints")

        try:
            while curr_idx < total:
                wp: RobotJoints = waypoints[curr_idx]
                joints = [wp.j1, wp.j2, wp.j3, wp.j4, wp.j5, wp.j6]

                is_last = curr_idx == total - 1
                userdef = "2" if is_last else "1"

                if goal_handle.is_cancel_requested:
                    userdef = "2"

                jointtarget_str = RWSInterface.joints_to_dipc_jointtarget(joints)

                retries = 0
                while True:
                    if goal_handle.is_cancel_requested and userdef != "2":
                        self.logger.info(
                            f"Cancel requested during retries at waypoint {curr_idx}/{total}"
                        )
                        userdef = "2"

                    _, status_code = self.RWS.send_dipc_message(
                        message=jointtarget_str, userdef=userdef
                    )

                    if status_code == 204:
                        break

                    if status_code == 500 and retries < dipc_retry_max:
                        retries += 1
                        time.sleep(dipc_retry_delay_s)
                        continue

                    error_msg = (
                        f"DIPC send failed at waypoint {curr_idx + 1}/{total}: "
                        f"(status={status_code}, retries={retries})"
                    )
                    self.logger.error(error_msg)
                    goal_handle.abort()
                    result.success = False
                    result.message = error_msg
                    result.executed_count = sent_count
                    return result

                sent_count += 1

                feedback_msg.current_index = curr_idx
                feedback_msg.state = f"Sent waypoint {curr_idx + 1}/{total}"
                goal_handle.publish_feedback(feedback_msg)

                curr_idx += 1

                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.success = False
                    result.message = f"Cancelled after sending {sent_count} waypoint(s)"
                    result.executed_count = sent_count
                    self.logger.info(result.message)
                    return result

            goal_handle.succeed()
            result.success = True
            result.message = f"All {total} waypoints sent successfully"
            result.executed_count = sent_count
            self.logger.info(result.message)
            return result

        except Exception as e:
            error_msg = f"Error during joint trajectory execution: {e}"
            self.logger.error(error_msg)
            goal_handle.abort()
            result.success = False
            result.message = error_msg
            result.executed_count = sent_count
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
            response.message, response.status_code = self.robot_request(cmd, params)
            response.status = True

        except Exception as e:
            error_msg = f"Error handling controller_request '{cmd}': {e}"
            self.logger.error(error_msg)
            response.message = error_msg
            response.status_code = -1
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

        # Create multithreaded executor with 2 threads
        executor = MultiThreadedExecutor(num_threads=2)
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
        except Exception:
            pass  # rclpy may already be shut down


if __name__ == "__main__":
    main()
