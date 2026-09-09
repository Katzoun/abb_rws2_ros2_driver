"""A failed operation must not report success because a read answered 200.

make_robot_ready is a sequence of requests, and any of them can fail while the
last one that answered was a perfectly good 200 from reading the state.
"""

from robot_control.rws.interface import RWSInterface
from robot_control.rws.provider import NO_STATUS, RWSResult


def test_a_result_still_unpacks_as_a_pair():
    message, status = RWSResult("OK", 204)
    assert (message, status) == ("OK", 204)


def test_any_2xx_is_ok():
    assert RWSResult("OK", 200).ok is True


def test_a_404_is_not_ok():
    assert RWSResult("gone", 404).ok is False


def test_no_status_is_not_ok():
    assert RWSResult("nothing", NO_STATUS).ok is False


def test_an_error_overrides_a_200():
    assert RWSResult.error("ERR - nope", 200).ok is False


def test_an_error_still_unpacks():
    message, status = RWSResult.error("ERR - nope", 200)
    assert (message, status) == ("ERR - nope", 200)


class FakeRWS:
    """Just enough of RWSInterface for make_robot_ready to run."""

    class _Log:
        def info(self, message):
            pass

        def error(self, message):
            pass

    logger = _Log()

    def __init__(
        self, opmode="auto", ctrl="motoron", running=False, execstate="running"
    ):
        self.opmode = opmode
        self.ctrl = ctrl
        self.running = running
        self.execstate = execstate

    def is_master(self, domain):
        return domain == "edit"

    def release_mastership(self, domain):
        return RWSResult("OK", 204)

    def is_running(self):
        return self.running

    def get_opmode_state(self):
        return RWSResult(self.opmode, 200)  # a read that succeeded

    def get_controller_state(self):
        return RWSResult(self.ctrl, 200)  # a read that succeeded

    def motors_on(self):
        return RWSResult("OK", 204)

    def reset_pp(self):
        return RWSResult("OK", 204)

    def start_rapid_script(self):
        return RWSResult("OK", 204)

    def get_rapid_execution_state(self):
        return RWSResult(f'[{{"ctrlexecstate": "{self.execstate}"}}]', 200)


def ready(**controller):
    return RWSInterface.make_robot_ready(FakeRWS(**controller))


def test_manual_mode_is_a_failure():
    assert ready(opmode="manual").ok is False


def test_motors_that_stay_off_are_a_failure():
    assert ready(ctrl="motoroff").ok is False


def test_rapid_that_will_not_start_is_a_failure():
    assert ready(execstate="stopped").ok is False


def test_the_healthy_path_still_succeeds():
    assert ready().ok is True


def test_an_already_running_robot_still_succeeds():
    assert ready(running=True).ok is True
