import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
import inspect
import json
import time

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from interface_pkg.action import ExecutePoseArray, ExecuteJointArray
from interface_pkg.msg import RobotJoints

from core_pkg.nodes_core import INTRANode, handle_operation_errors
from core_pkg.systemconstants import NodeStates
from core_pkg.systemconstants import RobotControllerConstants as RCC
from core_pkg.exceptions import RWSException, NodeExceptionRecoverable, NodeExceptionNonRecoverable

from intranodes_pkg.parameters.robot_controller_parameters import RobotParameters, RobotParametersKeys
from intranodes_pkg.robot_controller_interface import RWSInterface

from interface_pkg.srv import RobotRequestSrv, TriggerServiceSrv

from statemachine.exceptions import TransitionNotAllowed


class RobotControllerNode(INTRANode):

    def __init__(self):
        super().__init__(RCC.NODE_NAME)
        # Initialize default Robot parameters
        self.default_ros_params = RobotParameters().to_ros_params()

        self.RWS = None
        self._logged_in = False
        self.keepalive_timer = None
        self.joint_states_timer = None
        self.egm_active = False

        # Declare ROS2 parameters
        declared_params = self.declare_parameters(
            namespace='',  # Empty because params already have dot notation
            parameters=self.default_ros_params
        )
        self.logger.info(f"Declared {len(declared_params)} parameters")
        
        # Services
        self.controller_request_service = self.create_service(RobotRequestSrv,
        f"{RCC.NODE_NAME}/{RCC.ServiceNames.CONTROLLER_REQUEST}",
        self.controller_request_cb, callback_group=self.cb_group)

        # Topics
        self.joint_states_publisher = self.create_publisher(JointState, f"{RCC.NODE_NAME}/{RCC.TopicNames.JOINT_STATES_TOPIC}", 10)

        # Actions

        self.robot_robtarget_move_action_server = ActionServer(
            self,
            ExecutePoseArray,
            f"{RCC.NODE_NAME}/{RCC.ActionNames.ROBOT_ROBTARGET_MOVE_ACTION}",
            execute_callback=self.execute_pose_array_cb,
            goal_callback=self.goal_callback_pose,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group
        )

        self.robot_jointtarget_move_action_server = ActionServer(
            self,
            ExecuteJointArray,
            f"{RCC.NODE_NAME}/{RCC.ActionNames.ROBOT_JOINTTARGET_MOVE_ACTION}",
            execute_callback=self.execute_joint_array_cb,
            goal_callback=self.goal_callback_joint,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group
        )



        with self.sm_lock:
            self.lifecycle_sm.boot_complete(message=f"Boot of {self.NODE_NAME} complete, waiting for initialization")

    def apply_parameters(self):
        """Apply parameters to hardware"""
        pass

    def cleanup_resources(self):
        """Cleanup resources"""
        try:
            if self.keepalive_timer is not None:
                self.keepalive_timer.destroy()
                self.keepalive_timer = None
            if self.joint_states_timer is not None:
                self.joint_states_timer.destroy()
                self.joint_states_timer = None

            
            self.RWS.logout()
            self._logged_in = False

            # destroy interface object
            self.RWS = None
        except Exception:
            self.RWS = None
            self._logged_in = False
            self.keepalive_timer = None
            self.joint_states_timer = None

    def initialize_node(self):
        """Initialize robot controller node"""
        if self.RWS is not None or self._logged_in:
            raise NodeExceptionRecoverable('Node already initialized, cleanup before re-initializing')

        try:
            try:
                self.logger.info('Initializing RWSInterface...')
                self.RWS = RWSInterface(
                        host=self.get_parameter(RobotParametersKeys.IP_ADDRESS).value,
                        username=self.get_parameter(RobotParametersKeys.USERNAME).value,
                        password=self.get_parameter(RobotParametersKeys.PASSWORD).value,
                        port=self.get_parameter(RobotParametersKeys.PORT).value,
                        logger=self.logger
                    )
            except Exception as e:
                raise NodeExceptionNonRecoverable(f'Failed to initialize RWSInterface: {e}')
            
            try:#login to robot
                status = self.RWS.login()
                if status:
                    self._logged_in = True
                    self.logger.info('Login successful')
                else:
                    self._logged_in = False
            except RWSException as e:
                raise NodeExceptionNonRecoverable(f'Login failed: {e}')
        
            if self.get_parameter(RobotParametersKeys.SEND_KEEPALIVE).value:
                period = self.get_parameter(RobotParametersKeys.KEEPALIVE_INTERVAL).value
                self.keepalive_timer = self.create_timer(period, self.keepalive_callback, callback_group=self.cb_group)
            
            if self.get_parameter(RobotParametersKeys.SEND_JOINT_STATES).value:
                period = self.get_parameter(RobotParametersKeys.JOINT_STATES_HZ).value
                period = 1.0 / period if period > 0 else 1.0  # Convert Hz to seconds, default to 1 Hz if invalid
                self.joint_states_timer = self.create_timer(period, self.joint_states_callback, callback_group=self.cb_group)
            
            
        except Exception as e:
            raise NodeExceptionNonRecoverable(f'Error during node initialization: {e}')

    def keepalive_callback(self):
        if self._logged_in:
            self.logger.info('Sending keepalive signal...')
            self._logged_in = False
            try:
                result = self.RWS.send_keepalive()
                if result == True:
                    self._logged_in = True
                    self.logger.info('Keepalive successful')
                else:
                    self.logger.error('Keepalive failed: No response from robot')

            except Exception as e:
                self.logger.error(f'Keepalive failed: {e}')
        else:
            self.logger.info('Not logged in, skipping keepalive.')

    def joint_states_callback(self):
        if self._logged_in:
            try:
                if not self.egm_active:
                    result_json, status_code = self.RWS.get_robot_joint_positions()
                    if status_code != 200:
                        self.logger.warning(f"Failed to get joint states (status={status_code}): {result_json}")
                        return

                    data = json.loads(result_json)
                    joint_names = [
                        "Revolute 1",
                        "Revolute 2",
                        "Revolute 3",
                        "Revolute 4",
                        "Revolute 5",
                        "Revolute 6",
                    ]

                    positions_deg = []
                    for i in range(1, 7):
                        key = f"rax_{i}"
                        value = data.get(key, None)
                        if value is None:
                            raise ValueError(f"Missing joint value for {key}")
                        positions_deg.append(float(value))

                    msg = JointState()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.name = joint_names
                    msg.position = [float(np.deg2rad(v)) for v in positions_deg]

                    self.joint_states_publisher.publish(msg)

                else: 
                    self.logger.info('EGM active, skipping joint states retrieval to avoid conflicts.')
            except Exception as e:
                self.logger.error(f'Failed to get joint states: {e}')
        else:
            self.logger.info('Not logged in, skipping joint states retrieval.')

    # ============= ACTION SERVER CALLBACKS =============

    def goal_callback_pose(self, goal_request: ExecutePoseArray.Goal) -> GoalResponse:
        try:
            with self.sm_lock:
                self.lifecycle_sm.start_operation(
                    message=f"Received new goal with {len(goal_request.path.poses)} poses for DIPC trajectory execution")

            
            if len(goal_request.path.poses) == 0:
                self.logger.error('Goal rejected: empty PoseArray')
                with self.sm_lock:
                    self.lifecycle_sm.complete(
                        message=f"Rejected goal with empty PoseArray, going back to idle"
                    )
                return GoalResponse.REJECT
            
            if not self.RWS.is_rapid_idle():
                self.logger.error('Goal rejected: Robot is not in idle state')
                with self.sm_lock:
                    self.lifecycle_sm.complete(
                        message=f"Rejected goal because robot is not idle, going back to idle"
                    )
                return GoalResponse.REJECT
            
            self.logger.info(f'Goal accepted: {len(goal_request.path.poses)} poses to send via DIPC')
            return GoalResponse.ACCEPT
        
        except TransitionNotAllowed as e:
            self.logger.error(f'Transition not allowed: {e}')
            return GoalResponse.REJECT
        
    def cancel_callback(self, goal_handle):
        """Accept cancel requests."""
        self.logger.info('Cancel requested for DIPC trajectory execution')
        return CancelResponse.ACCEPT

    def execute_pose_array_cb(self, goal_handle: ServerGoalHandle) -> ExecutePoseArray.Result:
        """
        Execute callback for the DIPC trajectory action.
        """
        goal: ExecutePoseArray.Goal = goal_handle.request

        dipc_retry_max = self.get_parameter(RobotParametersKeys.DIPC_RETRY_MAX).value
        dipc_retry_delay_s = self.get_parameter(RobotParametersKeys.DIPC_RETRY_DELAY_S).value
        
        try:
            MC = RCC.MotionCommands
            if goal.motion_command not in [MC.MOVE_L, MC.MOVE_J]:
                raise ValueError(f"Unsupported motion command: {goal.motion_command}")

            if goal.motion_command == MC.MOVE_L:
                routine_name = RCC.Routines.MOVE_L
            elif goal.motion_command == MC.MOVE_J:
                routine_name = RCC.Routines.MOVE_J

            self.RWS.set_rapid_symbol_raw(f'"{routine_name}"', RCC.Symbols.ROUTINE_NAME, RCC.Modules.RAPID)
            time.sleep(0.1)
            self.RWS.set_rapid_symbol_raw(goal.speed, RCC.Symbols.SPEED, RCC.Modules.USER)
            time.sleep(0.1)
            self.RWS.set_rapid_symbol_raw(RCC.States.EXECUTE, RCC.Symbols.CURRENT_STATE, RCC.Modules.MAIN)
            time.sleep(0.3)
        
        except Exception as e:
            error_msg = f"Error in execute_pose_array_cb: {e}"
            self.logger.error(error_msg)
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

        self.logger.info(f'Starting DIPC trajectory execution with {total} poses')

        try:
            while curr_pose < total:
                pose = poses[curr_pose]

                # Determine userdef: 2 for last point, 1 otherwise.
                is_last = (curr_pose == total - 1)
                userdef = "2" if is_last else "1"

                # If a cancel has been requested, promote this point to the last point
                if goal_handle.is_cancel_requested:
                    userdef = "2"

                robtarget_str = RWSInterface.pose_to_dipc_robtarget(pose)

                retries = 0
                while True:
                    # If cancel arrives during retries
                    if goal_handle.is_cancel_requested and userdef != "2":
                        self.logger.info(f'Cancel requested during retries at pose {curr_pose}/{total}')
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

                    error_msg = (f"DIPC send failed at pose {curr_pose+1}/{total}: "
                                 f"(status={status_code}, retries={retries})")
                    self.logger.error(error_msg)
                    goal_handle.abort()
                    result.success = False
                    result.message = error_msg
                    result.executed_count = sent_count

                    with self.sm_lock:
                        self.lifecycle_sm.fail_recoverable(message=error_msg)
                    return result

                sent_count += 1

                feedback_msg.current_index = curr_pose
                feedback_msg.state = f"Sent pose {curr_pose+1}/{total}"
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

                    with self.sm_lock:
                        self.lifecycle_sm.complete(message=f"DIPC trajectory cancelled after {sent_count} pose(s)")
                    return result

            # All poses sent successfully
            goal_handle.succeed()
            result.success = True
            result.message = f"All {total} poses sent successfully"
            result.executed_count = sent_count

            with self.sm_lock:
                self.lifecycle_sm.complete(message=f"DIPC trajectory completed: {total} poses sent")

            self.logger.info(result.message)
            return result

        except Exception as e:
            error_msg = f"Error during DIPC trajectory execution: {e}"
            self.logger.error(error_msg)
            goal_handle.abort()
            result.success = False
            result.message = error_msg
            result.executed_count = sent_count

            with self.sm_lock:
                self.lifecycle_sm.fail_non_recoverable(message=error_msg)
            
            return result

    def goal_callback_joint(self, goal_request: ExecuteJointArray.Goal) -> GoalResponse:
        try:
            waypoints: list[RobotJoints] = goal_request.waypoints
            with self.sm_lock:
                self.lifecycle_sm.start_operation(
                    message=f"Received joint trajectory goal with {len(waypoints)} waypoints")

            if len(waypoints) == 0:
                self.logger.error('Goal rejected: empty joint trajectory')
                with self.sm_lock:
                    self.lifecycle_sm.complete(message="Rejected goal: empty joint trajectory")
                return GoalResponse.REJECT

            if not self.RWS.is_rapid_idle():
                self.logger.error('Goal rejected: robot not idle')
                with self.sm_lock:
                    self.lifecycle_sm.complete(message="Rejected goal: robot not idle")
                return GoalResponse.REJECT

            self.logger.info(f'Joint trajectory goal accepted: {len(waypoints)} waypoints')
            return GoalResponse.ACCEPT

        except TransitionNotAllowed as e:
            self.logger.error(f'Transition not allowed: {e}')
            return GoalResponse.REJECT
        except Exception as e:
            self.logger.error(f'Error in goal_callback_joint: {e}')
            return GoalResponse.REJECT

    def execute_joint_array_cb(self, goal_handle: ServerGoalHandle) -> ExecuteJointArray.Result:
        """
        Execute callback for the joint trajectory DIPC action.
        """
        goal: ExecuteJointArray.Goal = goal_handle.request

        dipc_retry_max = self.get_parameter(RobotParametersKeys.DIPC_RETRY_MAX).value
        dipc_retry_delay_s = self.get_parameter(RobotParametersKeys.DIPC_RETRY_DELAY_S).value

        try:
            waypoints: list[RobotJoints] = goal.waypoints

            if goal.motion_command not in [RCC.MotionCommands.MOVE_ABS_J, RCC.MotionCommands.MOVE_ABS_L]:
                raise ValueError(f"Unsupported motion command: {goal.motion_command}")

            if goal.motion_command == RCC.MotionCommands.MOVE_ABS_J:
                routine_name = RCC.Routines.MOVE_ABS_J
            elif goal.motion_command == RCC.MotionCommands.MOVE_ABS_L:
                routine_name = RCC.Routines.MOVE_ABS_L

            self.RWS.set_rapid_symbol_raw(f'"{routine_name}"', RCC.Symbols.ROUTINE_NAME, RCC.Modules.RAPID)
            time.sleep(0.1)
            self.RWS.set_rapid_symbol_raw(goal.speed, RCC.Symbols.SPEED, RCC.Modules.USER)
            time.sleep(0.1)
            self.RWS.set_rapid_symbol_raw(RCC.States.EXECUTE, RCC.Symbols.CURRENT_STATE, RCC.Modules.MAIN)
            time.sleep(0.3)

        except Exception as e:
            error_msg = f"Error in execute_joint_array_cb setup: {e}"
            self.logger.error(error_msg)
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

        self.logger.info(f'Starting DIPC joint trajectory with {total} waypoints')

        try:
            while curr_idx < total:
                wp: RobotJoints = waypoints[curr_idx]
                joints = [wp.j1, wp.j2, wp.j3, wp.j4, wp.j5, wp.j6]

                is_last = (curr_idx == total - 1)
                userdef = "2" if is_last else "1"

                if goal_handle.is_cancel_requested:
                    userdef = "2"

                jointtarget_str = RWSInterface.joints_to_dipc_jointtarget(joints)

                retries = 0
                while True:
                    if goal_handle.is_cancel_requested and userdef != "2":
                        self.logger.info(f'Cancel requested during retries at waypoint {curr_idx}/{total}')
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

                    error_msg = (f"DIPC send failed at waypoint {curr_idx+1}/{total}: "
                                 f"(status={status_code}, retries={retries})")
                    self.logger.error(error_msg)
                    goal_handle.abort()
                    result.success = False
                    result.message = error_msg
                    result.executed_count = sent_count
                    with self.sm_lock:
                        self.lifecycle_sm.fail_recoverable(message=error_msg)
                    return result

                sent_count += 1

                feedback_msg.current_index = curr_idx
                feedback_msg.state = f"Sent waypoint {curr_idx+1}/{total}"
                goal_handle.publish_feedback(feedback_msg)

                curr_idx += 1

                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.success = False
                    result.message = f"Cancelled after sending {sent_count} waypoint(s)"
                    result.executed_count = sent_count
                    with self.sm_lock:
                        self.lifecycle_sm.complete(
                            message=f"Joint trajectory cancelled after {sent_count} waypoint(s)")
                    return result

            goal_handle.succeed()
            result.success = True
            result.message = f"All {total} waypoints sent successfully"
            result.executed_count = sent_count
            with self.sm_lock:
                self.lifecycle_sm.complete(
                    message=f"Joint trajectory completed: {total} waypoints sent")
            self.logger.info(result.message)
            return result

        except Exception as e:
            error_msg = f"Error during joint trajectory execution: {e}"
            self.logger.error(error_msg)
            goal_handle.abort()
            result.success = False
            result.message = error_msg
            result.executed_count = sent_count
            with self.sm_lock:
                self.lifecycle_sm.fail_non_recoverable(message=error_msg)
            return result

    def controller_request_cb(self, request: RobotRequestSrv.Request, response : RobotRequestSrv.Response):
        """Handle GET request service call"""   
        params: list[str] = [param for param in request.params]
        #filter out falsy params
        params = [p for p in params if p]
        cmd: str = request.command

        self.logger.info(f"Received controller_request: command='{cmd}' params={params}")

        try:
            response.message, response.status_code = self.robot_request(cmd, params)
            response.status = True

        except Exception as e:
            error_msg = f"Error handling controller_request '{cmd}': {e}"
            self.logger.error(error_msg)
            response.message = error_msg
            response.status_code = -1
            response.status = False
        
        self.logger.info(f"Controller request response: status={response.status}  message='{response.message}'  status_code={response.status_code}")

        return response

    def robot_request(self, cmd: str, params: list):

        if self._logged_in and self.RWS is not None:
            if not hasattr(self.RWS, cmd):
                raise NodeExceptionRecoverable(f"Method {cmd} not found")

            method = getattr(self.RWS, cmd)
            signature = inspect.signature(method)
            # Filter out 'self' parameter
            method_params = [p for name, p in signature.parameters.items() if name != 'self']
            num_total_params = len(method_params)
            # Get number of required parameters
            num_required_params = sum(1 for p in method_params if p.default == inspect.Parameter.empty)

            try:
                if num_required_params == 0 and len(params) == 0:
                    # No parameters expected
                    return method()
                elif num_required_params == len(params):
                    # Exact match
                    return method(*params)
                elif len(params) <= num_total_params and len(params) >= num_required_params:
                    # Some optional parameters can be omitted
                    return method(*params)
                
                elif len(params) < num_required_params:
                    # Fewer parameters provided than required
                    raise NodeExceptionRecoverable(f"Not enough parameters provided: got {len(params)}, need at least {num_required_params}")
                else:
                    # Too many parameters, use only what's needed
                    return method(*params[:num_total_params])

            except TypeError as e:
                raise NodeExceptionRecoverable(f"Parameter mismatch calling {cmd}: {e}")
        else:
            raise NodeExceptionNonRecoverable("Node not initialized or not logged in to robot, reinitialize the node.")

def main(args=None):

    rclpy.init(args=args)
    robot_controller_node = None
    executor = None

    try:
        # Initialize the Robot Controller node
        robot_controller_node = RobotControllerNode()
        
        # Create multithreaded executor with 2 threads
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(robot_controller_node)
        
        # Spin the executor
        executor.spin()
        
    except KeyboardInterrupt:
        if robot_controller_node:
            robot_controller_node.logger.info("Keyboard interrupt received")
            
    except Exception as e:
        if robot_controller_node:
            robot_controller_node.logger.error(f"Unexpected error: {e}")
        else:
            print(f'Error during node initialization: {e}')
            
    finally:
        # Shutdown executor (stops spinning and waits for callbacks to finish)
        if executor:
            executor.shutdown(timeout_sec=5)
        
        # Update lifecycle state
        if robot_controller_node:
            with robot_controller_node.sm_lock:
                try:
                    robot_controller_node.lifecycle_sm.shutdown(
                        message="Node shutting down"
                    )
                except Exception as e:
                    robot_controller_node.logger.warning(
                        f"State transition failed during shutdown: {e}"
                    )
        
        # Cleanup node-specific resources
        if robot_controller_node:
            try:
                robot_controller_node.cleanup_resources()
            except Exception as e:
                robot_controller_node.logger.error(
                    f"Error during cleanup: {e}"
                )
        
        # Destroy the node
        if robot_controller_node:
            robot_controller_node.destroy_node()
        
        # Shutdown rclpy
        try:
            rclpy.shutdown()
        except Exception:
            pass  # rclpy may already be shut down

if __name__ == '__main__':
    main()