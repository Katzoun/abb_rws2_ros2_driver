"""The trajectory actions, driven against a scripted controller.

No middleware and no robot: the real action body runs against fakes, with a
clock the test moves by hand so a timeout can be reached instantly.
"""

import json

import pytest
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Pose, PoseArray

from robot_control import robot_controller_node as N
from robot_control.robot_controller_node import RobotControllerNode as Node
from robot_control_msgs.msg import RobotJoints


class Clock:
    """Stands in for the time module, so waiting costs nothing."""

    def __init__(self):
        self.now = 0.0

    def monotonic(self) -> float:
        return self.now

    def sleep(self, _seconds) -> None:
        pass


CLOCK = Clock()


@pytest.fixture(autouse=True)
def fake_clock(monkeypatch):
    CLOCK.now = 0.0
    monkeypatch.setattr(N, "time", CLOCK)


class Log:
    def info(self, message):
        pass

    def error(self, message):
        pass

    def warning(self, message):
        pass


class FakeRWS:
    """A controller whose answers follow a script, one step per poll."""

    def __init__(
        self,
        states=("2",),
        running=True,
        joints=None,
        tick=1.0,
        fail_at=None,
        accept=True,
        reject=False,
        completes_after=3,
        moves=None,
    ):
        self.states = list(states)  # current_state, one entry per poll
        self.running = running
        self.joints = joints  # j1 per poll; None means "keeps moving"
        self.tick = tick
        self.fail_at = fail_at  # 1-based index of the send that fails
        self.accept = accept  # does the routine confirm it started
        self.reject = reject  # does RAPID say the name is unknown
        self.completes_after = completes_after  # poll at which it reports done
        self.moves = moves  # moves_done per poll
        self.sent = []
        self.writes = []
        self.polls = 0
        self.request_id = 0

    def set_rapid_symbol_raw(self, value, symbol, module):
        self.writes.append((symbol, value))
        if symbol == "request_id":
            self.request_id = int(value)
        return ("OK", 204)

    def send_dipc_message(self, message, userdef):
        self.sent.append((message, userdef))
        if self.fail_at is not None and len(self.sent) == self.fail_at:
            return ("boom", 400)
        return ("OK", 204)

    def is_running(self):
        return self.running

    def get_rapid_symbol(self, symbol, module):
        if symbol == "accepted_id":
            # The only read the start wait makes, so the clock moves here too.
            CLOCK.now += self.tick
            return (str(self.request_id if self.accept else 0), 200)
        if symbol == "rejected_id":
            return (str(self.request_id if self.reject else 0), 200)
        if symbol == "completed_id":
            done = (
                self.completes_after is not None and self.polls >= self.completes_after
            )
            return (str(self.request_id if done else 0), 200)
        if symbol == "moves_done":
            if self.moves:
                return (str(self.moves[min(self.polls, len(self.moves) - 1)]), 200)
            return (str(self.polls), 200)
        return (self.states[min(self.polls, len(self.states) - 1)], 200)

    def get_robot_joint_positions(self):
        index = min(self.polls, len(self.joints) - 1) if self.joints else 0
        j1 = self.joints[index] if self.joints else float(self.polls)
        self.polls += 1
        CLOCK.now += self.tick
        angles = {f"rax_{axis}": "0" for axis in range(1, 7)}
        angles["rax_1"] = str(j1)
        return (json.dumps(angles), 200)


class FakeHandle:
    """A goal handle that can turn cancelled on a chosen read."""

    def __init__(self, waypoints, cancel_from=None, tool="", wobj=""):
        self.request = Goal(waypoints, tool, wobj)
        self.outcome = None
        self._reads = 0
        self._cancel_from = cancel_from

    @property
    def is_cancel_requested(self):
        self._reads += 1
        return self._cancel_from is not None and self._reads >= self._cancel_from

    def publish_feedback(self, message):
        pass

    def canceled(self):
        self.outcome = "canceled"

    def abort(self):
        self.outcome = "aborted"

    def succeed(self):
        self.outcome = "succeeded"


class Goal:
    def __init__(self, count, tool="", wobj=""):
        self.waypoints = [
            RobotJoints(**{f"j{axis}": float(i) for axis in range(1, 7)})
            for i in range(count)
        ]
        self.motion_command = "MoveAbsJ"
        self.speed = "100"
        self.tool = tool
        self.wobj = wobj


class FakeNode:
    def __init__(self, rws):
        self.RWS = rws
        self.logger = Log()
        self._active = True
        self._logged_in = True
        self._request_id = 0

    def get_clock(self):
        return self

    def now(self):
        return self

    def to_msg(self):
        return Time()

    def _param_int(self, key):
        return 1

    def _param_float(self, key):
        return {
            "motion.completion_timeout_s": 60.0,
            "motion.stall_timeout_s": 10.0,
            "motion.poll_interval_s": 0.5,
            "motion.start_timeout_s": 5.0,
            "dipc.health_check_s": 0.5,
        }.get(key, 0.0)

    _rapid_alive = Node._rapid_alive
    _rapid_running_quiet = Node._rapid_running_quiet
    _start_routine = Node._start_routine
    _end_buffer_routine = Node._end_buffer_routine
    _read_joints = Node._read_joints
    _wait_for_motion_end = Node._wait_for_motion_end
    _await_routine_start = Node._await_routine_start
    _next_request_id = Node._next_request_id
    _read_num = Node._read_num


def run(waypoints=3, cancel_from=None, tool="", wobj="", **controller):
    """One whole goal against a scripted controller."""
    rws = FakeRWS(**controller)
    handle = FakeHandle(waypoints, cancel_from=cancel_from, tool=tool, wobj=wobj)
    result = Node._execute_joint_array(FakeNode(rws), handle)
    return result, handle.outcome, rws


def userdefs(rws):
    return [userdef for _, userdef in rws.sent]


# --- finishing when the robot finishes --------------------------------------


def test_finishes_only_once_the_robot_confirms():
    result, outcome, _ = run(states=["2"] * 8, completes_after=3)
    assert outcome == "succeeded"
    assert result.success


def test_stopped_rapid_aborts_the_goal():
    result, outcome, _ = run(states=["2"], running=False, completes_after=None)
    assert outcome == "aborted"
    assert not result.success


def test_motionless_robot_trips_the_stall_watchdog():
    result, outcome, _ = run(
        states=["2"] * 40, joints=[5.0] * 40, completes_after=None, moves=[1] * 40
    )
    assert outcome == "aborted"
    assert "not moved" in result.message


def test_a_trajectory_that_never_ends_hits_the_timeout():
    result, outcome, _ = run(states=["2"] * 200, tick=5.0, completes_after=None)
    assert outcome == "aborted"
    assert "did not finish" in result.message


def test_deactivation_stops_the_wait():
    node = FakeNode(FakeRWS(states=["2"] * 20))
    node._active = False
    finished, message, _ = Node._wait_for_motion_end(
        node, node.RWS, 1, lambda done: None, 60.0, 10.0, 0.5
    )
    assert finished is False
    assert "deactivated" in message


def test_a_lost_session_stops_the_wait():
    node = FakeNode(FakeRWS(states=["2"] * 20))
    node._logged_in = False
    finished, message, _ = Node._wait_for_motion_end(
        node, node.RWS, 1, lambda done: None, 60.0, 10.0, 0.5
    )
    assert finished is False
    assert "session" in message


def test_a_finish_seen_just_before_idle_still_counts():
    """RAPID writes completed_id a moment before the state machine goes idle."""

    class RacingRWS(FakeRWS):
        def __init__(self, **kwargs):
            super().__init__(states=["0"] * 8, **kwargs)
            self.completed_reads = 0

        def get_rapid_symbol(self, symbol, module):
            if symbol == "completed_id":
                self.completed_reads += 1
                # Not on the poll's own read, but on the re-check after idle.
                return (str(self.request_id if self.completed_reads > 1 else 0), 200)
            return super().get_rapid_symbol(symbol, module)

    rws = RacingRWS(completes_after=None, moves=[7] * 8)
    handle = FakeHandle(7)
    result = Node._execute_joint_array(FakeNode(rws), handle)
    assert handle.outcome == "succeeded"
    assert result.executed_count == 7


# --- the routine has to confirm it started ----------------------------------


def test_an_unknown_routine_fails_instead_of_reporting_success():
    result, outcome, rws = run(
        waypoints=5, states=["0"] * 8, accept=False, reject=True, completes_after=None
    )
    assert outcome == "aborted"
    assert not result.success
    assert rws.sent == []


def test_no_confirmation_means_nothing_is_sent():
    _result, outcome, rws = run(
        waypoints=5, states=["2"] * 8, accept=False, completes_after=None
    )
    assert outcome == "aborted"
    assert rws.sent == []


def test_a_routine_that_ends_without_finishing_is_not_a_success():
    result, outcome, _ = run(states=["0"] * 8, completes_after=None)
    assert outcome == "aborted"
    assert not result.success


def test_request_ids_never_repeat_and_never_hit_zero():
    node = FakeNode(FakeRWS(states=["2"], completes_after=0))
    ids = [node._next_request_id() for _ in range(3)]
    assert ids == [1, 2, 3]


# --- every way out leaves a terminator behind -------------------------------


def test_a_clean_run_sends_exactly_one_terminator():
    _, outcome, rws = run(waypoints=3)
    assert outcome == "succeeded"
    assert userdefs(rws) == ["1", "1", "2"]


def test_cancel_landing_right_after_a_flyby_send_still_terminates():
    """The cancel becomes visible only once waypoint 2 went out as a fly-by."""
    result, outcome, rws = run(waypoints=5, cancel_from=6, states=["2"] * 8)
    assert outcome == "canceled"
    assert userdefs(rws).count("2") == 1
    assert "Cancelled" in result.message


def test_cancel_pending_before_the_first_send():
    _, outcome, rws = run(waypoints=5, cancel_from=1)
    assert outcome == "canceled"
    assert userdefs(rws) == ["2"]


def test_cancel_waits_for_the_queue_to_drain():
    result, outcome, _rws = run(
        waypoints=5, cancel_from=6, states=["2"] * 8, completes_after=2
    )
    assert outcome == "canceled"
    # The count comes from RAPID, not from how many points we pushed.
    assert result.executed_count == 2


def test_a_rejected_send_still_terminates_then_aborts():
    result, outcome, rws = run(
        waypoints=5, states=["2"] * 8, completes_after=1, fail_at=2
    )
    assert outcome == "aborted"
    assert userdefs(rws) == ["1", "1", "2"]
    assert "DIPC send failed" in result.message


# --- a stopped RAPID is noticed while sending, not after -------------------


class StoppingRWS(FakeRWS):
    """RAPID falls over partway through the send, as it does on a bad point.

    Sending costs time on a real controller, so the clock moves here too -
    that is what lets the health check come due.
    """

    def __init__(self, stops_after=1, **kwargs):
        super().__init__(**kwargs)
        self.stops_after = stops_after

    def send_dipc_message(self, message, userdef):
        CLOCK.now += 0.12  # roughly one DIPC round-trip
        if len(self.sent) >= self.stops_after:
            self.running = False
        return super().send_dipc_message(message, userdef)


def test_a_rapid_that_stops_mid_send_ends_the_goal_without_sending_the_rest():
    """The whole point of the health check: fail in a moment, not in 30 s."""
    rws = StoppingRWS(stops_after=1, states=["2"] * 8, completes_after=None)
    handle = FakeHandle(249)
    result = Node._execute_joint_array(FakeNode(rws), handle)

    assert handle.outcome == "aborted"
    # Without the check every one of the 249 points would have gone out first.
    assert len(rws.sent) < 10
    assert "RAPID stopped while sending" in result.message


def test_the_send_error_survives_the_wait_message():
    rws = StoppingRWS(stops_after=1, states=["2"] * 8, completes_after=None)
    result = Node._execute_joint_array(FakeNode(rws), FakeHandle(20))

    # The wait fails too, and used to be the only thing reported.
    assert "RAPID stopped while sending" in result.message
    assert "RAPID stopped before the trajectory finished" in result.message


def test_releasing_the_buffer_is_skipped_once_rapid_has_stopped():
    node = FakeNode(FakeRWS(running=False))
    Node._end_buffer_routine(node, node.RWS, "jointtarget;[[0]]", 100, 0.25)
    assert node.RWS.sent == []


def test_a_running_rapid_still_gets_its_terminator():
    node = FakeNode(FakeRWS(running=True))
    Node._end_buffer_routine(node, node.RWS, "jointtarget;[[0]]", 100, 0.25)
    assert userdefs(node.RWS) == ["2"]


def test_an_unreadable_running_state_does_not_abort_the_send():
    """One failed read is not a stopped robot."""

    class GrumpyRWS(FakeRWS):
        def is_running(self):
            raise RuntimeError("controller said 503")

    node = FakeNode(GrumpyRWS())
    alive, _ = Node._rapid_alive(node, node.RWS, 0.0, 0.0)
    assert alive is True

    # And the terminator still goes out, rather than being skipped on a guess.
    Node._end_buffer_routine(node, node.RWS, "jointtarget;[[0]]", 100, 0.25)
    assert userdefs(node.RWS) == ["2"]


# --- tool and workobject ----------------------------------------------------


def symbol_writes(rws):
    return dict(rws.writes)


def test_tool_and_wobj_names_reach_the_controller():
    _result, _outcome, rws = run(
        waypoints=3, tool="tooltuzka", wobj="wobjtabletop", completes_after=1
    )
    written = symbol_writes(rws)
    assert written["tool_name_input"] == '"tooltuzka"'
    assert written["wobj_name_input"] == '"wobjtabletop"'


def test_an_empty_tool_is_still_written():
    """Skipping the write would leave the last goal's tool standing."""
    _result, _outcome, rws = run(waypoints=3, completes_after=1)
    written = symbol_writes(rws)
    assert written["tool_name_input"] == '""'
    assert written["wobj_name_input"] == '""'


def test_the_tool_goes_out_before_the_routine_starts():
    _result, _outcome, rws = run(waypoints=3, completes_after=1)
    symbols = [symbol for symbol, _ in rws.writes]
    assert symbols.index("tool_name_input") < symbols.index("current_state")
    assert symbols.index("wobj_name_input") < symbols.index("current_state")


def test_a_rejected_tool_names_all_three_in_the_message():
    result, outcome, _rws = run(
        waypoints=3,
        tool="nosuchtool",
        states=["0"] * 8,
        accept=False,
        reject=True,
        completes_after=None,
    )
    assert outcome == "aborted"
    assert "nosuchtool" in result.message
    assert "rejected" in result.message


@pytest.mark.parametrize("name", ['t"Pen', "tool;drop", "tool name", "9tool", "a" * 33])
def test_names_that_would_break_the_symbol_write_are_not_names(name):
    assert not N.valid_rapid_name(name)


@pytest.mark.parametrize("name", ["", "tooltuzka", "wobj_dock1", "T0"])
def test_the_names_actually_on_the_controller_pass(name):
    assert N.valid_rapid_name(name)


# --- the cartesian path, which had no coverage at all -----------------------


class PoseGoal:
    def __init__(self, count, tool="", wobj=""):
        self.path = PoseArray(poses=[Pose() for _ in range(count)])
        self.motion_command = "MoveL"
        self.speed = "100"
        self.tool = tool
        self.wobj = wobj


class PoseHandle(FakeHandle):
    def __init__(self, count, cancel_from=None, tool="", wobj=""):
        super().__init__(0, cancel_from=cancel_from)
        self.request = PoseGoal(count, tool, wobj)


def run_pose(count=3, **controller):
    rws = FakeRWS(**controller)
    handle = PoseHandle(count)
    return Node._execute_pose_array(FakeNode(rws), handle), handle.outcome, rws


def test_a_cartesian_path_finishes_when_the_robot_confirms():
    result, outcome, rws = run_pose(3, states=["2"] * 8, completes_after=3)
    assert outcome == "succeeded"
    assert result.success
    assert userdefs(rws) == ["1", "1", "2"]


def test_a_cartesian_path_stops_sending_once_rapid_stops():
    rws = StoppingRWS(stops_after=1, states=["2"] * 8, completes_after=None)
    handle = PoseHandle(249)
    result = Node._execute_pose_array(FakeNode(rws), handle)

    assert handle.outcome == "aborted"
    assert len(rws.sent) < 10
    assert "RAPID stopped while sending at pose" in result.message


def test_a_cartesian_goal_passes_its_tool_and_wobj_on():
    rws = FakeRWS(states=["2"] * 8, completes_after=1)
    handle = PoseHandle(3, tool="tooltuzka", wobj="wobjtabletop")
    Node._execute_pose_array(FakeNode(rws), handle)

    written = symbol_writes(rws)
    assert written["tool_name_input"] == '"tooltuzka"'
    assert written["wobj_name_input"] == '"wobjtabletop"'
