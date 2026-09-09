"""What the controller_request service is allowed to call.

This table is the interface. A method that is not listed here cannot be
reached from ROS, and the summary next to each name is what the service
answers with when asked for help.

The rule behind the list: every read is exposed, because reading changes
nothing; every mutation is listed one by one and is refused while a
trajectory is running, unless it is the deliberate way to interrupt one;
the session and raw-HTTP plumbing is not exposed at all.
"""

from collections.abc import Sequence
from dataclasses import dataclass

HELP_COMMAND = "help"


@dataclass(frozen=True)
class Command:
    """One RWS method, as the service exposes it."""

    summary: str
    args: tuple[str, ...] = ()
    optional: tuple[str, ...] = ()
    while_moving: bool = False


# Deliberately absent, so nobody has to wonder whether it was forgotten:
#   login, logout, get_login_state, send_keepalive - the node owns the session
#   get_request, post_request, options_request, get_generic, get_generic_json,
#     get_embed_json - any HTTP call to any path, which is no interface at all
#   set_rapid_symbol_raw, apply_rapid_writes - could rewrite the handshake
#     variables or the state machine underneath a running goal
#   send_dipc_message, create_dipc_queue, read_dipc_message - would inject or
#     steal points from a trajectory the actions are streaming
#   is_running, is_rapid_idle, is_master - they answer with a bare bool, which
#     the service cannot tell apart from a failure; the get_ twins are listed
COMMANDS: dict[str, Command] = {
    # Reading, always allowed.
    "get_clock": Command("Controller clock.", while_moving=True),
    "get_controller_state": Command(
        "Motors on or off, guardstop, emergencystop or init.", while_moving=True
    ),
    "get_opmode_state": Command(
        "Operating mode: auto, man or manf.", while_moving=True
    ),
    "get_safety_mode": Command("Safety mode of the controller.", while_moving=True),
    "get_speedratio": Command("Global speed override, 0-100.", while_moving=True),
    "get_robot_type": Command(
        "Robot model, for example CRB 15000-10/1.52.", while_moving=True
    ),
    "get_system_options": Command("Installed system options.", while_moving=True),
    "get_system_products": Command("Installed system products.", while_moving=True),
    "get_energy_info": Command("Accumulated energy figures.", while_moving=True),
    "get_leadthrough_state": Command(
        "Whether lead-through is active.",
        optional=("mechunit_name",),
        while_moving=True,
    ),
    "get_robot_baseframe": Command(
        "Base frame of the mechanical unit.",
        optional=("mechunit_name",),
        while_moving=True,
    ),
    "get_robot_cartesian": Command(
        "Current cartesian position.", optional=("mechunit_name",), while_moving=True
    ),
    "get_robot_robtarget": Command(
        "Current robtarget, the pose the actions speak in.",
        optional=("mechunit_name",),
        while_moving=True,
    ),
    "get_robot_jointtarget": Command(
        "Current jointtarget, six angles in degrees.",
        optional=("mechunit_name",),
        while_moving=True,
    ),
    "get_robot_joint_positions": Command(
        "The six joint angles as a flat JSON object.",
        optional=("mechunit_name",),
        while_moving=True,
    ),
    "get_rapid_execution_state": Command(
        "Whether the RAPID program is running or stopped.", while_moving=True
    ),
    "get_rapid_idle": Command(
        "True when the RAPID state machine is idle and ready for a routine.",
        while_moving=True,
    ),
    "get_rapid_tasks": Command("RAPID tasks on the controller.", while_moving=True),
    "get_task_robtarget": Command(
        "Robtarget of one RAPID task.", optional=("task_name",), while_moving=True
    ),
    "get_task_jointtarget": Command(
        "Jointtarget of one RAPID task.", optional=("task_name",), while_moving=True
    ),
    "get_task_modules": Command(
        "Modules loaded in one RAPID task.", optional=("task_name",), while_moving=True
    ),
    "get_rapid_symbol": Command(
        "Value of a RAPID variable, for example current_state in TRobMain.",
        args=("symbol_name", "module_name"),
        optional=("task_name",),
        while_moving=True,
    ),
    "get_rapid_symbol_properties": Command(
        "Type and scope of a RAPID variable.",
        args=("symbol_name", "module_name"),
        optional=("task_name",),
        while_moving=True,
    ),
    "get_rapid_retcode": Command(
        "What an ABB error number means, for example -1073445859.",
        args=("retcode_name",),
        while_moving=True,
    ),
    "get_io_networks": Command("IO networks on the controller.", while_moving=True),
    "get_io_signals": Command("All IO signals.", while_moving=True),
    "get_io_signal": Command(
        "State of one IO signal. Most resolve without a network and device.",
        args=("signal_name",),
        optional=("network", "device"),
        while_moving=True,
    ),
    "get_dipc_queues": Command("The DIPC queues that exist.", while_moving=True),
    "get_dipc_queue_info": Command(
        "Configuration of one DIPC queue. queue-size is its capacity, not its fill.",
        optional=("queue_name",),
        while_moving=True,
    ),
    "get_mastership_state": Command(
        "Who holds mastership of the edit or motion domain.",
        args=("domain",),
        while_moving=True,
    ),
    "get_user_uas": Command("Grants held by the logged-in user.", while_moving=True),
    "get_all_grants": Command("Every grant the controller knows.", while_moving=True),
    # Mutating. Refused while a trajectory is running, except where noted.
    "make_robot_ready": Command(
        "Mastership, motors on, RAPID started - the whole preparation."
    ),
    "motors_on": Command("Turn the motors on."),
    "motors_off": Command("Turn the motors off."),
    "start_rapid_script": Command("Start RAPID execution."),
    "stop_rapid_script": Command(
        "Stop RAPID. Allowed mid-trajectory: the running goal aborts.",
        while_moving=True,
    ),
    "reset_pp": Command("Move the program pointer back to main."),
    "restart_controller": Command(
        "Restart the controller. Everything in flight is lost."
    ),
    "set_speedratio": Command(
        "Set the global speed override, 0-100. Allowed mid-trajectory.",
        args=("speed_ratio",),
        while_moving=True,
    ),
    "set_io_signal": Command(
        "Set one IO signal.",
        args=("signal_name", "signal_value"),
        optional=("network", "device"),
    ),
    "request_mastership": Command(
        "Take mastership of a domain, or both when left out.", optional=("domain",)
    ),
    "release_mastership": Command(
        "Give mastership back. The node retakes what it needs.", optional=("domain",)
    ),
    "reset_energy_info": Command("Zero the accumulated energy figures."),
    "run_rapid_routine": Command(
        "Start a RAPID routine by name, for example take_tool1. Returns once it "
        "has started, not once it has finished.",
        args=("routine_name",),
    ),
}


def parse_params(name: str, command: Command, params: Sequence[str]) -> dict[str, str]:
    """Turn ['key=value', ...] into keyword arguments for the method."""
    kwargs: dict[str, str] = {}

    for param in params:
        key, separator, value = param.partition("=")
        if not separator:
            raise ValueError(f"Parameter '{param}' is not in name=value form")
        key = key.strip()
        if key in kwargs:
            raise ValueError(f"Parameter '{key}' given twice")
        if key not in command.args and key not in command.optional:
            raise ValueError(f"'{name}' takes no parameter '{key}'")
        kwargs[key] = value

    missing = [arg for arg in command.args if arg not in kwargs]
    if missing:
        raise ValueError(f"'{name}' needs {', '.join(missing)}")

    return kwargs


def render_help() -> str:
    """The table as text, so the interface can be read without the source."""
    lines = [
        "controller_request calls one RWS method by name.",
        "params are name=value, in any order: ['signal_name=DO_1', 'signal_value=1'].",
        "A * marks what may be called while a trajectory is running.",
        "",
    ]

    for name, command in COMMANDS.items():
        arguments = ", ".join(
            list(command.args) + [f"[{option}]" for option in command.optional]
        )
        mark = "*" if command.while_moving else " "
        lines.append(f"{mark} {f'{name}({arguments})'.ljust(55)} {command.summary}")

    lines.append("")
    lines.append(f"* {HELP_COMMAND.ljust(55)} This list.")
    return "\n".join(lines)
