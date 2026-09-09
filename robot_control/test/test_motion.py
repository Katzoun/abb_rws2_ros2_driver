"""The trajectory actions, driven against a scripted controller.

No middleware and no robot: the real action body runs against fakes, with a
clock the test moves by hand so a timeout can be reached instantly.
"""

import json

import pytest

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
        self.polls = 0
        self.request_id = 0

    def set_rapid_symbol_raw(self, value, symbol, module):
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

    def __init__(self, waypoints, cancel_from=None):
        self.request = Goal(waypoints)
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
    def __init__(self, count):
        self.waypoints = [
            RobotJoints(**{f"j{axis}": float(i) for axis in range(1, 7)})
            for i in range(count)
        ]
        self.motion_command = "MoveAbsJ"
        self.speed = "100"


class FakeNode:
    def __init__(self, rws):
        self.RWS = rws
        self.logger = Log()
        self._active = True
        self._logged_in = True
        self._request_id = 0

    def _param_int(self, key):
        return 1

    def _param_float(self, key):
        return {
            "motion.completion_timeout_s": 60.0,
            "motion.stall_timeout_s": 10.0,
            "motion.poll_interval_s": 0.5,
            "motion.start_timeout_s": 5.0,
        }.get(key, 0.0)

    _start_routine = Node._start_routine
    _end_buffer_routine = Node._end_buffer_routine
    _read_joints = Node._read_joints
    _wait_for_motion_end = Node._wait_for_motion_end
    _await_routine_start = Node._await_routine_start
    _next_request_id = Node._next_request_id
    _read_num = Node._read_num


def run(waypoints=3, cancel_from=None, **controller):
    """One whole goal against a scripted controller."""
    rws = FakeRWS(**controller)
    handle = FakeHandle(waypoints, cancel_from=cancel_from)
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
