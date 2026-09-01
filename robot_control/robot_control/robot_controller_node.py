"""The RWS side of the system: a managed node that owns the robot session.

It comes up unconfigured and does nothing until driven through the lifecycle.
Configure logs in to the controller, activate starts the keepalive and joint
state timers and opens it up for motion goals.
"""

import json
import time
from math import radians

import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import Node as LifecycleNode, State, TransitionCallbackReturn

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

from robot_control_msgs.action import ExecutePoseArray, ExecuteJointArray
from robot_control_msgs.srv import RobotRequestSrv

from robot_control.constants import RobotControllerConstants as RCC
from robot_control.rws.interface import RWSInterface
from robot_control.rws.simulated import SimulatedRWS

NODE_NAME = "robot_controller"


class RobotControllerNode(LifecycleNode):

    # Which RAPID routine runs a given motion command. The routine names
    # themselves are parameters, so the RAPID side can keep its own naming.
    ROUTINE_PARAM = {
        RCC.MotionCommands.MOVE_L: 'routines.buffer_move_l',
        RCC.MotionCommands.MOVE_J: 'routines.buffer_move_j',
        RCC.MotionCommands.MOVE_ABS_J: 'routines.buffer_move_abs_j',
        RCC.MotionCommands.MOVE_ABS_L: 'routines.buffer_move_abs_l',
    }

    def __init__(self):
        super().__init__(NODE_NAME)
        self.logger = self.get_logger()
        self.cb_group = ReentrantCallbackGroup()

        self.RWS = None
        self._logged_in = False
        self.keepalive_timer = None
        self.joint_states_timer = None
        # rclpy exposes no public accessor for the current lifecycle state, so
        # track it ourselves for the goal callbacks to check.
        self._active = False

        self.declare_parameters('', [
            # Connection. The launch file overrides these from .env; the values
            # here are what the physical controller in the lab uses. Backend
            # 'rws' talks to a real controller, 'sim' keeps everything in
            # process so the pipeline can be exercised without a robot.
            ('connection.backend', 'rws'),
            ('connection.ip_address', '192.168.0.37'),
            ('connection.port', 443),
            ('connection.username', 'Admin'),
            ('connection.password', 'robotics'),

            ('utility.send_keepalive', True),
            ('utility.keepalive_interval', 30.0),
            ('utility.send_joint_states', True),
            ('utility.joint_states_hz', 10.0),

            # How hard to push when the RAPID buffer queue answers 500, meaning
            # it is full and the robot has not drained it yet.
            ('dipc.retry_max', 20),
            ('dipc.retry_delay_s', 0.1),

            ('routines.buffer_move_l', RCC.Routines.MOVE_L),
            ('routines.buffer_move_j', RCC.Routines.MOVE_J),
            ('routines.buffer_move_abs_j', RCC.Routines.MOVE_ABS_J),
            ('routines.buffer_move_abs_l', RCC.Routines.MOVE_ABS_L),
        ])

        self.create_service(
            RobotRequestSrv, f"{NODE_NAME}/controller_request",
            self.controller_request_cb, callback_group=self.cb_group)

        self.joint_states_publisher = self.create_publisher(
            JointState, f"{NODE_NAME}/joint_states", 10)

        ActionServer(
            self, ExecutePoseArray, f"{NODE_NAME}/robot_robtarget_move",
            execute_callback=self.execute_pose_array_cb,
            goal_callback=self.goal_callback_pose,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group)

        ActionServer(
            self, ExecuteJointArray, f"{NODE_NAME}/robot_jointtarget_move",
            execute_callback=self.execute_joint_array_cb,
            goal_callback=self.goal_callback_joint,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group)

        self.logger.info(f'{NODE_NAME} is unconfigured - configure it to connect to the robot')

    # ============= LIFECYCLE =============

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """Open the RWS session. Timers stay off until we are activated."""
        if self.RWS is not None:
            self.logger.error('Already configured, clean up first')
            return TransitionCallbackReturn.FAILURE

        backend = self.get_parameter('connection.backend').value
        if backend not in ('rws', 'sim'):
            self.logger.error(f"Unknown backend {backend!r}, expected 'rws' or 'sim'")
            return TransitionCallbackReturn.FAILURE

        if backend == 'sim':
            self.logger.warning('Using the simulated controller - no robot will move')
            self.RWS = SimulatedRWS(logger=self.logger)
            self.RWS.login()
            self._logged_in = True
            return TransitionCallbackReturn.SUCCESS

        try:
            self.logger.info('Initializing RWSInterface...')
            self.RWS = RWSInterface(
                host=self.get_parameter('connection.ip_address').value,
                username=self.get_parameter('connection.username').value,
                password=self.get_parameter('connection.password').value,
                port=self.get_parameter('connection.port').value,
                logger=self.logger,
            )
            if not self.RWS.login():
                self.logger.error('Login rejected by the controller')
                self.RWS = None
                return TransitionCallbackReturn.FAILURE
        except Exception as e:
            self.logger.error(f'Could not reach the robot: {e}')
            self.RWS = None
            return TransitionCallbackReturn.FAILURE

        self._logged_in = True
        self.logger.info('Login successful')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Start the background chatter with the controller."""
        if self.get_parameter('utility.send_keepalive').value:
            self.keepalive_timer = self.create_timer(
                self.get_parameter('utility.keepalive_interval').value,
                self.keepalive_callback, callback_group=self.cb_group)

        if self.get_parameter('utility.send_joint_states').value:
            hz = self.get_parameter('utility.joint_states_hz').value
            self.joint_states_timer = self.create_timer(
                1.0 / hz if hz > 0 else 1.0,
                self.joint_states_callback, callback_group=self.cb_group)

        self._active = True
        self.logger.info('Active - accepting motion goals')
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Stop the timers but keep the session open."""
        self._active = False
        self._stop_timers()
        self.logger.info('Inactive - motion goals will be rejected')
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self._release()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self._release()
        return TransitionCallbackReturn.SUCCESS

    def on_error(self, state: State) -> TransitionCallbackReturn:
        self.logger.error('Transition failed, dropping the RWS session')
        self._release()
        return TransitionCallbackReturn.SUCCESS

    def _stop_timers(self):
        for name in ('keepalive_timer', 'joint_states_timer'):
            timer = getattr(self, name)
            if timer is not None:
                timer.destroy()
                setattr(self, name, None)

    def _release(self):
        """Drop timers and the RWS session, tolerating a robot that is already gone."""
        self._active = False
        self._stop_timers()
        try:
            if self.RWS is not None:
                self.RWS.logout()
        except Exception as e:
            self.logger.warning(f'Logout failed, dropping the session anyway: {e}')
        finally:
            self.RWS = None
            self._logged_in = False

    def keepalive_callback(self):
        if not self._logged_in:
            return
        try:
            self._logged_in = bool(self.RWS.send_keepalive())
            if not self._logged_in:
                self.logger.error('Keepalive rejected - the session is gone')
        except Exception as e:
            self._logged_in = False
            self.logger.error(f'Keepalive failed: {e}')

    def joint_states_callback(self):
        if not self._logged_in:
            return
        try:
            result_json, status_code = self.RWS.get_robot_joint_positions()
            if status_code != 200:
                self.logger.warning(f"Failed to get joint states (status={status_code}): {result_json}")
                return

            data = json.loads(result_json)
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = [f"Revolute {i}" for i in range(1, 7)]
            msg.position = [radians(float(data[f"rax_{i}"])) for i in range(1, 7)]
            self.joint_states_publisher.publish(msg)
        except Exception as e:
            self.logger.error(f'Failed to get joint states: {e}')

    # ============= ACTION SERVER CALLBACKS =============

    def cancel_callback(self, goal_handle):
        self.logger.info('Cancel requested for DIPC trajectory execution')
        return CancelResponse.ACCEPT

    def _accept_goal(self, count: int, noun: str) -> GoalResponse:
        """Both motion actions are accepted on the same three conditions."""
        if not self._active:
            self.logger.error('Goal rejected: node is not active')
            return GoalResponse.REJECT

        if count == 0:
            self.logger.error(f'Goal rejected: no {noun}s in the goal')
            return GoalResponse.REJECT

        try:
            if not self.RWS.is_rapid_idle():
                self.logger.error('Goal rejected: robot is not in idle state')
                return GoalResponse.REJECT
        except Exception as e:
            self.logger.error(f'Goal rejected: could not read robot state: {e}')
            return GoalResponse.REJECT

        self.logger.info(f'Goal accepted: {count} {noun}s to send via DIPC')
        return GoalResponse.ACCEPT

    def goal_callback_pose(self, goal_request: ExecutePoseArray.Goal) -> GoalResponse:
        return self._accept_goal(len(goal_request.path.poses), 'pose')

    def goal_callback_joint(self, goal_request: ExecuteJointArray.Goal) -> GoalResponse:
        return self._accept_goal(len(goal_request.waypoints), 'waypoint')

    def execute_pose_array_cb(self, goal_handle: ServerGoalHandle) -> ExecutePoseArray.Result:
        poses = goal_handle.request.path.poses
        targets = [RWSInterface.pose_to_dipc_robtarget(p) for p in poses]

        def feedback(index):
            msg = ExecutePoseArray.Feedback()
            msg.current_pose = PoseStamped()
            msg.current_pose.header.stamp = self.get_clock().now().to_msg()
            msg.current_pose.pose = poses[index]
            return msg

        return self._stream_to_dipc(
            goal_handle, ExecutePoseArray.Result(), targets, feedback, 'pose')

    def execute_joint_array_cb(self, goal_handle: ServerGoalHandle) -> ExecuteJointArray.Result:
        targets = [
            RWSInterface.joints_to_dipc_jointtarget([w.j1, w.j2, w.j3, w.j4, w.j5, w.j6])
            for w in goal_handle.request.waypoints
        ]
        return self._stream_to_dipc(
            goal_handle, ExecuteJointArray.Result(), targets,
            lambda index: ExecuteJointArray.Feedback(), 'waypoint')

    def _stream_to_dipc(self, goal_handle, result, targets, make_feedback, noun):
        """Push a whole path into the RAPID buffer queue, one target at a time.

        Both motion actions land here - they differ only in the string handed to
        the controller and in the feedback message they publish.
        """
        goal = goal_handle.request
        result.executed_count = 0

        routine_param = self.ROUTINE_PARAM.get(goal.motion_command)
        if routine_param is None:
            return self._abort(goal_handle, result,
                               f"Unsupported motion command: {goal.motion_command}")

        try:
            self._start_routine(self.get_parameter(routine_param).value, goal.speed)
        except Exception as e:
            return self._abort(goal_handle, result, f"Could not arm the RAPID routine: {e}")

        retry_max = self.get_parameter('dipc.retry_max').value
        retry_delay = self.get_parameter('dipc.retry_delay_s').value
        total = len(targets)
        self.logger.info(f'Starting DIPC trajectory execution with {total} {noun}s')

        try:
            for index, target in enumerate(targets):
                # userdef marks the last point of the stream: 2 on the final
                # target, 1 on the rest. The RAPID side has to watch for it to
                # know the path has ended - tvarometr's program was written
                # against a trailing pen-up point instead, so this is one of the
                # things to line up on the controller. A cancel is delivered by
                # promoting whatever point is in flight to the last one.
                userdef = "2" if index == total - 1 else "1"

                for attempt in range(retry_max + 1):
                    if goal_handle.is_cancel_requested:
                        userdef = "2"

                    _, status_code = self.RWS.send_dipc_message(message=target, userdef=userdef)
                    if status_code == 204:
                        break
                    # 500 means the queue is full - wait for the robot to drain it.
                    if status_code != 500 or attempt == retry_max:
                        return self._abort(
                            goal_handle, result,
                            f"DIPC send failed at {noun} {index+1}/{total}: "
                            f"(status={status_code}, retries={attempt})")
                    time.sleep(retry_delay)

                result.executed_count = index + 1

                msg = make_feedback(index)
                msg.current_index = index
                msg.state = f"Sent {noun} {index+1}/{total}"
                goal_handle.publish_feedback(msg)

                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.success = False
                    result.message = f"Cancelled after sending {result.executed_count} {noun}(s)"
                    self.logger.info(result.message)
                    return result

            goal_handle.succeed()
            result.success = True
            result.message = f"All {total} {noun}s sent successfully"
            self.logger.info(result.message)
            return result

        except Exception as e:
            return self._abort(goal_handle, result, f"Error during DIPC execution: {e}")

    def _start_routine(self, routine_name: str, speed):
        """Point the RAPID program at a routine and let it run.

        The sleeps are the controller's, not ours: each symbol has to land
        before the next one goes out, or the routine starts on stale values.
        """
        self.RWS.set_rapid_symbol_raw(f'"{routine_name}"', RCC.Symbols.ROUTINE_NAME, RCC.Modules.RAPID)
        time.sleep(0.1)
        self.RWS.set_rapid_symbol_raw(speed, RCC.Symbols.SPEED, RCC.Modules.USER)
        time.sleep(0.1)
        self.RWS.set_rapid_symbol_raw(RCC.States.EXECUTE, RCC.Symbols.CURRENT_STATE, RCC.Modules.MAIN)
        time.sleep(0.3)

    def _abort(self, goal_handle, result, message):
        self.logger.error(message)
        goal_handle.abort()
        result.success = False
        result.message = message
        return result

    # ============= SERVICE =============

    def controller_request_cb(self, request: RobotRequestSrv.Request, response: RobotRequestSrv.Response):
        params = [p for p in request.params if p]
        cmd = request.command
        self.logger.info(f"Received controller_request: command='{cmd}' params={params}")

        try:
            response.message, response.status_code = self.robot_request(cmd, params)
            response.status = True
        except Exception as e:
            response.message = f"Error handling controller_request '{cmd}': {e}"
            response.status_code = -1
            response.status = False
            self.logger.error(response.message)

        self.logger.info(f"Controller request response: status={response.status}  "
                         f"message='{response.message}'  status_code={response.status_code}")
        return response

    def robot_request(self, cmd: str, params: list):
        """Call an RWS method by name.

        The command vocabulary is whatever RWSInterface exposes, which keeps the
        service useful for anything the orchestrator has not been taught yet.
        """
        if self.RWS is None or not self._logged_in:
            raise RuntimeError('Not logged in to the robot, configure the node first')

        method = getattr(self.RWS, cmd, None)
        if cmd.startswith('_') or not callable(method):
            raise RuntimeError(f"No such robot command: {cmd}")

        try:
            return method(*params)
        except TypeError as e:
            raise RuntimeError(f"Wrong parameters for {cmd}: {e}") from e


def main(args=None):
    rclpy.init(args=args)
    node = RobotControllerNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.logger.info("Keyboard interrupt received")
    finally:
        # Ctrl+C reaches the whole process group, and ros2 launch forwards it on
        # top of that, so a second interrupt lands while we are still tidying up.
        # Without this it turns a clean shutdown into a traceback.
        try:
            executor.shutdown(timeout_sec=5)
            # Close the RWS session however we got here.
            node._release()
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
