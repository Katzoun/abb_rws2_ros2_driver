"""What the request service will and will not call.

The table decides; this drives the node side of it, including the rule that a
command which changes something waits for the trajectory to finish.
"""

import threading

import pytest

from robot_control.robot_controller_node import RobotControllerNode as Node
from robot_control.rws.provider import RWSResult
from robot_control_msgs.srv import RobotRequestSrv


class Log:
    def info(self, message):
        pass

    def error(self, message):
        pass


class FakeRWS:
    def get_speedratio(self):
        return RWSResult("100", 200)

    def make_robot_ready(self):
        # A failure whose last request was a perfectly good read.
        return RWSResult.error("ERR - controller is in manual mode", 200)

    def get_rapid_symbol(self, symbol_name, module_name, task_name="T_ROB1"):
        return RWSResult(f"{module_name}/{symbol_name}", 200)

    def reset_energy_info(self):
        return RWSResult("OK", 204)


class FakeNode:
    def __init__(self, rws=None, logged_in=True, busy=False):
        self.RWS = rws
        self._logged_in = logged_in
        self._busy = busy
        self._busy_lock = threading.Lock()
        self.logger = Log()

    robot_request = Node.robot_request
    controller_request_cb = Node.controller_request_cb


def call(node, command, params=()):
    request = RobotRequestSrv.Request()
    request.command = command
    request.params = list(params)
    return node.controller_request_cb(request, RobotRequestSrv.Response())


def test_a_failure_carrying_200_is_still_a_failure():
    response = call(FakeNode(FakeRWS()), "make_robot_ready")
    assert response.status is False
    assert response.status_code == 200


def test_a_success_is_reported_as_one():
    response = call(FakeNode(FakeRWS()), "get_speedratio")
    assert response.status is True
    assert response.message == "100"


def test_named_parameters_reach_the_method():
    response = call(
        FakeNode(FakeRWS()),
        "get_rapid_symbol",
        ["module_name=TRobMain", "symbol_name=current_state"],
    )
    assert response.status is True
    assert response.message == "TRobMain/current_state"


@pytest.mark.parametrize(
    "command",
    [
        "wobble",  # no such thing
        "post_request",  # plumbing: an HTTP call to any path
        "set_rapid_symbol_raw",  # could rewrite the handshake
        "send_dipc_message",  # would inject a point into a trajectory
        "is_running",  # answers with a bare bool
        "logout",  # the node owns the session
    ],
)
def test_what_the_table_leaves_out_cannot_be_called(command):
    response = call(FakeNode(FakeRWS()), command)
    assert response.status is False
    assert "Unknown command" in response.message


@pytest.mark.parametrize(
    "params",
    [
        ["symbol_name=current_state"],  # module_name missing
        ["symbol_name=x", "module_name=y", "colour=red"],  # no such parameter
        ["current_state", "TRobMain"],  # positional, not name=value
    ],
)
def test_bad_parameters_are_refused(params):
    response = call(FakeNode(FakeRWS()), "get_rapid_symbol", params)
    assert response.status is False


def test_a_mutation_is_refused_while_a_trajectory_runs():
    response = call(FakeNode(FakeRWS(), busy=True), "reset_energy_info")
    assert response.status is False
    assert "refused" in response.message


def test_a_read_is_allowed_while_a_trajectory_runs():
    response = call(FakeNode(FakeRWS(), busy=True), "get_speedratio")
    assert response.status is True


def test_help_answers_without_a_session():
    """It describes the interface, so it must work before configure too."""
    response = call(FakeNode(rws=None, logged_in=False), "help")
    assert response.status is True
    assert "get_clock" in response.message


def test_a_call_without_a_session_is_refused():
    response = call(FakeNode(rws=None, logged_in=False), "get_speedratio")
    assert response.status is False
