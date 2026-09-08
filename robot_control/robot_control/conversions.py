"""Conversions between ROS messages and the text forms RAPID expects.

This is the one module in the package that speaks both languages. Everything
under rws/ deals in HTTP and strings and knows nothing about ROS, and keeping
the translation here is what lets rws/interface.py stay importable without a
ROS environment underneath it.
"""

from collections.abc import Sequence

from geometry_msgs.msg import Pose

# The arm has no external axes. 9E+09 is how ABB spells "not used".
_EXTAX = "9E+09,9E+09,9E+09,9E+09,9E+09,9E+09"

# Configuration data is left at zero: the RAPID side picks the arm
# configuration itself.
_CONFDATA = "[0,0,0,0]"


def _pose_fields(pose: Pose) -> tuple[str, str]:
    """Position and orientation of a pose in ABB's text form.

    A ROS pose is metres, an ABB robtarget millimetres, and the RAPID side does
    no scaling - so it happens here. ABB writes a quaternion w,x,y,z where ROS
    writes it x,y,z,w.
    """
    p = pose.position
    o = pose.orientation
    position = f"[{p.x * 1000:.3f},{p.y * 1000:.3f},{p.z * 1000:.3f}]"
    orientation = f"[{o.w:.6f},{o.x:.6f},{o.y:.6f},{o.z:.6f}]"
    return position, orientation


def pose_to_robtarget(pose: Pose) -> str:
    """A pose as a bare ABB robtarget."""
    position, orientation = _pose_fields(pose)
    return f"[{position},{orientation},{_CONFDATA},[{_EXTAX}]]"


def pose_list_to_robtargets(poses: Sequence[Pose]) -> str:
    """A list of poses as an ABB robtargets array."""
    return f"[{','.join(pose_to_robtarget(pose) for pose in poses)}]"


def pose_to_dipc_robtarget(pose: Pose) -> str:
    """A pose as a robtarget tagged for the DIPC queue."""
    position, orientation = _pose_fields(pose)
    return f"robtarget;[{position},{orientation},{_CONFDATA},[{_EXTAX}]]"


def joints_to_dipc_jointtarget(joints: Sequence[float]) -> str:
    """Six joint angles in degrees as a jointtarget for the DIPC queue.

    Format: jointtarget;[[j1..j6],[external axes]]
    """
    robax = ",".join(f"{joint:.4f}" for joint in joints[:6])
    return f"jointtarget;[[{robax}],[{_EXTAX}]]"
