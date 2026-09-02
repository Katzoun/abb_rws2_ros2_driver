"""The RWS side of the system: a managed node that owns the robot session.

It comes up unconfigured and does nothing until driven through the lifecycle.
Configure logs in to the controller, activate starts the keepalive and joint
state timers and opens it up for motion goals.
"""

import json
import threading
import time
from math import radians

import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import Node as LifecycleNode, State, TransitionCallbackReturn

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

from robot_control_msgs.action import ExecutePoseArray, ExecuteJointArray
from robot_control_msgs.srv import RobotRequestSrv

from robot_control.exceptions import RWSError
from robot_control.constants import (
    MotionCommands,
    ParamKeys as PK,
    RapidConfig,
    RoutineNames,
    default_joint_names,
)
from robot_control.rws.interface import RWSInterface
from robot_control.rws.simulated import SimulatedRWS

NODE_NAME = "robot_controller"


class RobotControllerNode(LifecycleNode):

    # Which RAPID routine runs a given motion command. The routine names
    # themselves are parameters, so the RAPID side can keep its own naming.
    ROUTINE_FOR = {
        MotionCommands.MOVE_L: 'buffer_move_l',
        MotionCommands.MOVE_J: 'buffer_move_j',
        MotionCommands.MOVE_ABS_J: 'buffer_move_abs_j',
        MotionCommands.MOVE_ABS_L: 'buffer_move_abs_l',
    }

    def __init__(self):
        super().__init__(NODE_NAME)
        self.logger = self.get_logger()

        # Motion actions stay reentrant: an ActionServer takes one group for all
        # of its callbacks, and a mutually exclusive one would leave no thread to
        # answer a cancel while the execute callback is streaming. Concurrency
        # between goals is kept out by the busy claim below, not by the group.
        self.motion_group = ReentrantCallbackGroup()
        # Timers and the request service get their own group so a long stream
        # cannot starve joint states, the keepalive, or a manual RWS call.
        self.util_group = MutuallyExclusiveCallbackGroup()

        self.RWS = None
        self._logged_in = False
        self.keepalive_timer = None
        self.joint_states_timer = None
        # rclpy exposes no public accessor for the current lifecycle state, so
        # track it ourselves for the goal callbacks to check.
        self._active = False

        # The robot is one resource and the RAPID buffer queue is one queue, so
        # only one motion goal may be in flight. Without this two goals overwrite
        # each other's routine and speed symbols and then interleave their points
        # into the same queue, which the robot happily draws as one path.
        self._busy_lock = threading.Lock()
        self._busy = False

        # Filled in by on_configure, from the joint_names parameter.
        self._joint_names = []

        self.declare_parameters('', [
            # Connection. The launch file overrides these from .env; the values
            # here are what the physical controller in the lab uses. Backend
            # 'rws' talks to a real controller, 'sim' keeps everything in
            # process so the pipeline can be exercised without a robot.
            (PK.BACKEND, 'rws'),
            (PK.IP_ADDRESS, '192.168.0.37'),
            (PK.PORT, 443),
            (PK.USERNAME, 'Admin'),
            (PK.PASSWORD, 'robotics'),
            # Raise for a controller that is not on the same switch.
            (PK.HTTP_TIMEOUT_S, 2.0),

            (PK.SEND_KEEPALIVE, True),
            (PK.KEEPALIVE_INTERVAL, 30.0),
            (PK.SEND_JOINT_STATES, True),
            (PK.JOINT_STATES_HZ, 10.0),

            # How hard to push when the RAPID buffer queue answers 500, meaning
            # it is full and the robot has not drained it yet.
            (PK.DIPC_RETRY_MAX, 20),
            (PK.DIPC_RETRY_DELAY_S, 0.1),
        ])

        self.create_service(
            RobotRequestSrv, f"{NODE_NAME}/controller_request",
            self.controller_request_cb, callback_group=self.util_group)

        self.joint_states_publisher = self.create_publisher(
            JointState, f"{NODE_NAME}/joint_states", 10)

        ActionServer(
            self, ExecutePoseArray, f"{NODE_NAME}/robot_robtarget_move",
            execute_callback=self.execute_pose_array_cb,
            goal_callback=self.goal_callback_pose,
            cancel_callback=self.cancel_callback,
            callback_group=self.motion_group)

        ActionServer(
            self, ExecuteJointArray, f"{NODE_NAME}/robot_jointtarget_move",
            execute_callback=self.execute_joint_array_cb,
            goal_callback=self.goal_callback_joint,
            cancel_callback=self.cancel_callback,
            callback_group=self.motion_group)

        self.logger.info(f'{NODE_NAME} is unconfigured - configure it to connect to the robot')

    # ============= LIFECYCLE =============

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """Open the RWS session. Timers stay off until we are activated."""
        if self.RWS is not None:
            self.logger.error('Already configured, clean up first')
            return TransitionCallbackReturn.FAILURE

        backend = self.get_parameter(PK.BACKEND).value
        if backend not in ('rws', 'sim'):
            self.logger.error(f"Unknown backend {backend!r}, expected 'rws' or 'sim'")
            return TransitionCallbackReturn.FAILURE

        rapid, routines = RapidConfig(), RoutineNames()
        self._joint_names = default_joint_names(rapid.num_axes)

        if backend == 'sim':
            self.logger.warning('Using the simulated controller - no robot will move')
            self.RWS = SimulatedRWS(logger=self.logger, rapid=rapid, routines=routines)
            self.RWS.login()
            self._logged_in = True
            return TransitionCallbackReturn.SUCCESS

        try:
            self.logger.info('Initializing RWSInterface...')
            self.RWS = RWSInterface(
                host=self.get_parameter(PK.IP_ADDRESS).value,
                username=self.get_parameter(PK.USERNAME).value,
                password=self.get_parameter(PK.PASSWORD).value,
                port=self.get_parameter(PK.PORT).value,
                logger=self.logger,
                timeout_s=self.get_parameter(PK.HTTP_TIMEOUT_S).value,
                rapid=rapid,
                routines=routines,
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
        if self.get_parameter(PK.SEND_KEEPALIVE).value:
            self.keepalive_timer = self.create_timer(
                self.get_parameter(PK.KEEPALIVE_INTERVAL).value,
                self.keepalive_callback, callback_group=self.util_group)

        if self.get_parameter(PK.SEND_JOINT_STATES).value:
            hz = self.get_parameter(PK.JOINT_STATES_HZ).value
            self.joint_states_timer = self.create_timer(
                1.0 / hz if hz > 0 else 1.0,
                self.joint_states_callback, callback_group=self.util_group)

        self._active = True
        self.logger.info('Active - accepting motion goals')
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Stop the timers but keep the session open.

        A goal already streaming is left to finish - stopping mid-path would
        leave the RAPID side waiting for an end marker that never comes.
        """
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
        # Whatever was in flight is over once the session goes; a claim left
        # standing here would refuse every goal after the next configure.
        self._free_robot()
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
            msg.name = self._joint_names
            msg.position = [radians(float(data[f"rax_{i}"]))
                            for i in range(1, len(self._joint_names) + 1)]
            self.joint_states_publisher.publish(msg)
        except Exception as e:
            self.logger.error(f'Failed to get joint states: {e}')

    # ============= ACTION SERVER CALLBACKS =============

    def cancel_callback(self, goal_handle):
        self.logger.info('Cancel requested for DIPC trajectory execution')
        return CancelResponse.ACCEPT

    def _claim_robot(self) -> bool:
        """Take the robot for one motion goal. False means someone else has it.

        Claiming and checking have to happen under the same lock: two goals
        arriving together would otherwise both read 'free' and both be accepted.
        """
        with self._busy_lock:
            if self._busy:
                return False
            self._busy = True
            return True

    def _free_robot(self):
        with self._busy_lock:
            self._busy = False

    def _accept_goal(self, count: int, noun: str) -> GoalResponse:
        """Both motion actions are accepted on the same four conditions.

        The claim is taken here rather than in the execute callback because
        rclpy accepts the next goal while the previous one is still streaming -
        by the time execution starts it is already too late to say no.
        """
        if not self._active:
            self.logger.error('Goal rejected: node is not active')
            return GoalResponse.REJECT

        if count == 0:
            self.logger.error(f'Goal rejected: no {noun}s in the goal')
            return GoalResponse.REJECT

        # Cheapest check that talks to nobody, so do it before the HTTP one.
        if not self._claim_robot():
            self.logger.error('Goal rejected: another motion goal is already running')
            return GoalResponse.REJECT

        try:
            if not self.RWS.is_rapid_idle():
                self.logger.error('Goal rejected: robot is not in idle state')
                self._free_robot()
                return GoalResponse.REJECT
        except Exception as e:
            self.logger.error(f'Goal rejected: could not read robot state: {e}')
            self._free_robot()
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

        Runs holding the busy claim its goal callback took, and hands it back on
        every way out of here.
        """
        try:
            return self._stream_to_dipc_locked(
                goal_handle, result, targets, make_feedback, noun)
        finally:
            self._free_robot()

    def _stream_to_dipc_locked(self, goal_handle, result, targets, make_feedback, noun):
        goal = goal_handle.request
        result.executed_count = 0

        # A shutdown mid-path drops self.RWS, so work from a local reference
        # rather than reaching for the attribute on every point.
        rws = self.RWS
        if rws is None:
            return self._abort(goal_handle, result, "No robot session")

        routine_field = self.ROUTINE_FOR.get(goal.motion_command)
        if routine_field is None:
            return self._abort(goal_handle, result,
                               f"Unsupported motion command: {goal.motion_command}")

        try:
            self._start_routine(rws, getattr(rws.routines, routine_field), goal.speed)
        except Exception as e:
            return self._abort(goal_handle, result, f"Could not arm the RAPID routine: {e}")

        retry_max = self.get_parameter(PK.DIPC_RETRY_MAX).value
        retry_delay = self.get_parameter(PK.DIPC_RETRY_DELAY_S).value
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

                    # False means the queue is full: normal back-pressure while
                    # the robot works through the path. A real failure raises.
                    if rws.send_dipc_message(message=target, userdef=userdef):
                        break
                    if attempt == retry_max:
                        return self._abort(
                            goal_handle, result,
                            f"DIPC queue still full at {noun} {index+1}/{total} "
                            f"after {retry_max} retries")
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

    def _start_routine(self, rws, routine_name: str, speed):
        """Point the RAPID program at a routine and let it run.

        The sleeps are the controller's, not ours: each symbol has to land
        before the next one goes out, or the routine starts on stale values.
        Only ever reached while holding the busy claim - these three symbols are
        global on the RAPID side, so a second goal writing them concurrently
        would redirect the running routine.
        """
        cfg = rws.rapid
        rws.set_rapid_symbol_raw(f'"{routine_name}"', cfg.symbol_routine_name, cfg.module_rapid)
        time.sleep(cfg.symbol_settle_s)
        rws.set_rapid_symbol_raw(speed, cfg.symbol_speed, cfg.module_user)
        time.sleep(cfg.symbol_settle_s)
        rws.set_rapid_symbol_raw(cfg.state_execute, cfg.symbol_current_state, cfg.module_main)
        time.sleep(cfg.state_settle_s)

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
        except RWSError as e:
            response.message = f"{cmd}: {e}"
            response.status_code = e.status_code
            response.status = False
            self.logger.error(response.message)
        except Exception as e:
            response.message = f"{cmd}: {e}"
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
            result = method(*params)
        except TypeError as e:
            raise RuntimeError(f"Wrong parameters for {cmd}: {e}") from e

        # Most methods now return the value asked for, or nothing at all, while
        # the service answers with (message, status). Getting there is this one
        # conversion rather than a tuple threaded through the whole driver.
        if isinstance(result, tuple) and len(result) == 2:
            return result
        return ("OK" if result is None else str(result), 200)


def main(args=None):
    rclpy.init(args=args)
    node = RobotControllerNode()
    # One thread streams the motion goal, one answers its cancel, and the
    # remaining two serve the util group - timers and the request service - so a
    # long path cannot starve the joint states or lock out a manual RWS call.
    executor = MultiThreadedExecutor(num_threads=4)
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
