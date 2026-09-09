"""Joint angles change units on the way to the robot.

A goal is in radians like the rest of ROS, a RAPID jointtarget is in degrees,
and getting this backwards moves the arm by a 57th of what was asked.
"""

from math import pi, radians

from robot_control.conversions import joints_to_dipc_jointtarget


def axes(jointtarget: str) -> list[float]:
    """The six angles back out of jointtarget;[[j1..j6],[ext axes]]."""
    robax = jointtarget.split("[[")[1].split("]")[0]
    return [float(value) for value in robax.split(",")]


def test_a_quarter_turn_reaches_rapid_as_ninety_degrees():
    assert axes(joints_to_dipc_jointtarget([pi / 2] * 6)) == [90.0] * 6


def test_zero_stays_zero():
    assert axes(joints_to_dipc_jointtarget([0.0] * 6)) == [0.0] * 6


def test_negative_angles_keep_their_sign():
    assert axes(joints_to_dipc_jointtarget([radians(-33.5)] * 6)) == [-33.5] * 6


def test_each_axis_keeps_its_place():
    joints = [radians(angle) for angle in (10.0, -20.0, 30.0, -40.0, 50.0, -60.0)]
    assert axes(joints_to_dipc_jointtarget(joints)) == [
        10.0,
        -20.0,
        30.0,
        -40.0,
        50.0,
        -60.0,
    ]


def test_external_axes_stay_unused():
    assert "9E+09" in joints_to_dipc_jointtarget([0.0] * 6)
