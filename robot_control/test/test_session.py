"""A healthy keepalive must never make the session look lost.

The flag it touches is read by the action while the robot is moving, so a
window where a working session reads as gone would abort a good goal.
"""

import threading
import time

from robot_control.robot_controller_node import RobotControllerNode as Node


class Log:
    def info(self, message):
        pass

    def error(self, message):
        pass


class FakeNode:
    def __init__(self, rws, logged_in=True):
        self.RWS = rws
        self._logged_in = logged_in
        self.logger = Log()

    keepalive_callback = Node.keepalive_callback


class SlowRWS:
    """A keepalive that succeeds, but takes its time about it."""

    def __init__(self, delay=0.4):
        self.delay = delay
        self.entered = threading.Event()
        self.calls = 0

    def send_keepalive(self):
        self.calls += 1
        self.entered.set()
        time.sleep(self.delay)
        return True


class RWS:
    def __init__(self, result=None, raises=False):
        self.result = result
        self.raises = raises
        self.calls = 0

    def send_keepalive(self):
        self.calls += 1
        if self.raises:
            raise ConnectionError("controller went away")
        return self.result


def test_a_slow_keepalive_never_reads_as_logged_out():
    rws = SlowRWS()
    node = FakeNode(rws)
    worker = threading.Thread(target=node.keepalive_callback)

    worker.start()
    rws.entered.wait(2.0)
    seen = []
    for _ in range(20):  # sample across the whole request
        seen.append(node._logged_in)
        time.sleep(0.02)
    worker.join()

    assert all(seen)
    assert node._logged_in


def test_a_refused_keepalive_drops_the_flag():
    node = FakeNode(RWS(result=False))
    node.keepalive_callback()
    assert node._logged_in is False


def test_a_transport_error_drops_the_flag():
    node = FakeNode(RWS(raises=True))
    node.keepalive_callback()
    assert node._logged_in is False


def test_no_session_object_means_not_logged_in():
    node = FakeNode(None)
    node.keepalive_callback()
    assert node._logged_in is False


def test_an_already_logged_out_node_does_not_call_the_controller():
    rws = RWS(result=True)
    node = FakeNode(rws, logged_in=False)
    node.keepalive_callback()
    assert node._logged_in is False
    assert rws.calls == 0
