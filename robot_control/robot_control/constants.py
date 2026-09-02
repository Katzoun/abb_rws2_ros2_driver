"""What the driver has to agree on with the robot, and the names of the
parameters that let a project change it.

Nothing here is a constant in the sense of being fixed - these are the defaults
for the lab's GoFa and the RAPID program it runs. Another installation names its
task, its mechanical unit and its symbols differently, so everything below is
reachable through ROS parameters; see config/robot_control.yaml.
"""

from dataclasses import dataclass
from typing import List


class MotionCommands:
    """Accepted values of the `motion_command` field in the motion actions.

    This one really is fixed: it is the driver's own vocabulary, not the
    controller's, and it is part of the action interface.
    """

    MOVE_L = "MoveL"
    MOVE_J = "MoveJ"
    MOVE_ABS_J = "MoveAbsJ"
    MOVE_ABS_L = "MoveAbsL"


@dataclass(frozen=True)
class RapidConfig:
    """The RAPID-side vocabulary, as one object handed to the RWS layer.

    Not ROS parameters: the driver and the RAPID program on the controller are a
    matched pair. Renaming a symbol here without changing the program does not
    make the driver work against a different one, it just breaks it. A project
    that ships its own RAPID program constructs its own RapidConfig instead.

    Passed in at construction rather than read per call, so a caller that omits
    the task or queue argument still talks to the right one.
    """

    # Where the program runs.
    task: str = "T_ROB1"
    mechunit: str = "ROB_1"
    dipc_queue: str = "RMQ_T_ROB1"
    num_axes: int = 6

    # Modules holding the symbols below.
    module_rapid: str = "TRobRAPID"
    module_user: str = "TRobUser"
    module_main: str = "TRobMain"

    # Symbols the driver writes to drive the program.
    symbol_routine_name: str = "routine_name_input"
    symbol_speed: str = "speednum"
    symbol_current_state: str = "current_state"
    symbol_received_robtarget: str = "received_robtarget"

    # Values of symbol_current_state.
    state_execute: str = "2"
    state_idle: str = "0"

    # The program needs each symbol to land before the next one goes out, and
    # needs to see the state change before it acts on it. Longer links and
    # slower controllers need more.
    symbol_settle_s: float = 0.1
    state_settle_s: float = 0.3
    # How long to give the controller to act on motors-on, a program pointer
    # reset or a program start before checking whether it took.
    controller_settle_s: float = 1.0


@dataclass(frozen=True)
class RoutineNames:
    """RAPID routines the driver triggers by name.

    Constants for the same reason as RapidConfig: a routine the program does not
    define cannot be run, so this is part of the contract with it rather than
    something to tune from a YAML file.
    """

    buffer_move_l: str = "run_routine_buffer_moveL"
    buffer_move_j: str = "run_routine_buffer_moveJ"
    buffer_move_abs_j: str = "run_routine_buffer_moveabsJ"
    buffer_move_abs_l: str = "run_routine_buffer_moveabsL"
    single_move_l: str = "run_single_moveL"
    single_move_j: str = "run_single_moveJ"


def default_joint_names(num_axes: int = 6) -> List[str]:
    """Joint names matching the ABB URDF convention.

    Worth overriding: these have to match the robot model that
    robot_state_publisher is fed, or nothing lines up with the joint states.
    """
    return [f"joint_{i}" for i in range(1, num_axes + 1)]


@dataclass(frozen=True)
class ParamKeys:
    """ROS parameter names, in one place so a typo fails at import, not at run."""

    BACKEND = "connection.backend"
    IP_ADDRESS = "connection.ip_address"
    PORT = "connection.port"
    USERNAME = "connection.username"
    PASSWORD = "connection.password"
    HTTP_TIMEOUT_S = "connection.http_timeout_s"

    SEND_KEEPALIVE = "utility.send_keepalive"
    KEEPALIVE_INTERVAL = "utility.keepalive_interval"
    SEND_JOINT_STATES = "utility.send_joint_states"
    JOINT_STATES_HZ = "utility.joint_states_hz"

    DIPC_RETRY_MAX = "dipc.retry_max"
    DIPC_RETRY_DELAY_S = "dipc.retry_delay_s"

