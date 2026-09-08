"""High-level ABB Robot Web Services interface
contains the endpoints and their payloads.
"""

import json
import time
from collections.abc import Sequence
from typing import Any

from robot_control.constants import RobotControllerConstants
from robot_control.rws.provider import NO_STATUS, RWSClient, RWSResult, SupportsLogging


class RWSInterface(RWSClient):
    """High-level interface for ABB Robot Web Services (RWS).
    Wraps HTTP requests to the robot controller for
    state, RAPID, IO, and DIPC operations.

    Unless said otherwise, a method returns an RWSResult - see its docstring
    for what the two fields hold and how to test for success."""

    def __init__(
        self,
        host: str,
        username: str,
        password: str,
        port: int = 80,
        logger: SupportsLogging | None = None,
    ) -> None:

        super().__init__(host, username, password, port=port, logger=logger)

    # A failed GET still has a body, but it describes the error and carries no
    # "state" - so check the status before reaching for the payload.
    def get_generic(self, endpoint: str, feature: str) -> RWSResult:
        (data, status) = self.get_request(endpoint)
        if status != 200 or data is None:
            return RWSResult(f"ERR - GET {endpoint} answered {status}", status)

        state = data.get("state") or []
        if not state:
            return RWSResult(f"ERR - {endpoint} returned no state", NO_STATUS)

        return RWSResult(state[0].get(feature, "unknown").lower(), status)

    def get_embed_json(self, endpoint: str) -> RWSResult:
        (data, status) = self.get_request(endpoint)
        if status != 200 or data is None:
            return RWSResult(f"ERR - GET {endpoint} answered {status}", status)

        resources = data.get("_embedded", {}).get("resources", [])
        return RWSResult(json.dumps(resources, indent=2), status)

    def get_generic_json(self, endpoint: str) -> RWSResult:
        (data, status) = self.get_request(endpoint)
        if status != 200 or data is None:
            return RWSResult(f"ERR - GET {endpoint} answered {status}", status)

        state = data.get("state")
        if state is None:
            return RWSResult(f"ERR - {endpoint} returned no state", NO_STATUS)

        return RWSResult(json.dumps(state, indent=2), status)

    def get_clock(self) -> RWSResult:
        """The controller clock."""
        return self.get_generic("/ctrl/clock", "datetime")

    def get_controller_state(self) -> RWSResult:
        """Controller state: motors on/off, guardstop, emergencystop or init."""
        return self.get_generic("/rw/panel/ctrl-state", "ctrlstate")

    def get_opmode_state(self) -> RWSResult:
        """Operational mode: auto, man or manf."""
        return self.get_generic("/rw/panel/opmode", "opmode")

    def get_safety_mode(self) -> RWSResult:
        """Returns (The current safety mode of the robot, http_status_code)."""
        return self.get_generic("/ctrl/safety/mode", "safetymode")

    # These three return the parsed body, not an RWSResult like the rest.
    def get_rapid_retcode(self, retcode_name: str) -> tuple[Any | None, int]:
        """Returns (RAPID return code value, http_status_code)."""
        if not retcode_name:
            raise ValueError("Return code name cannot be empty")
        return self.get_request(f"/rw/retcode/?code={retcode_name}")

    def get_user_uas(self) -> tuple[Any | None, int]:
        """Returns (the user-defined UAS variables, http_status_code)."""
        return self.get_request("/uas/user/grants")

    def get_all_grants(self) -> tuple[Any | None, int]:
        """Returns (the user-defined UAS variables, http_status_code)."""
        return self.get_request("/uas/grants")

    def get_speedratio(self) -> RWSResult:
        """Returns (The current speed ratio of the robot, http_status_code)."""
        return self.get_generic("/rw/panel/speedratio", "speedratio")

    def get_robot_type(self) -> RWSResult:
        """Robot type, for example CRB 15000-10/1.52."""
        return self.get_generic("/rw/system/robottype", "robot-type")

    def get_system_options(self) -> RWSResult:
        """System options, as a JSON string."""
        return self.get_generic_json("/rw/system/options")

    def get_system_products(self) -> RWSResult:
        """System products, as a JSON string."""
        return self.get_generic_json("/rw/system/products")

    def get_energy_info(self) -> RWSResult:
        """Energy information, as a JSON string."""
        return self.get_generic_json("/rw/system/energy")

    def get_leadthrough_state(self, mechunit_name: str = "ROB_1") -> RWSResult:
        """Returns (The leadthrough state of the robot, http_status_code)."""
        return self.get_generic(
            f"/rw/motionsystem/mechunits/{mechunit_name}/lead-through", "status"
        )

    def get_robot_baseframe(self, mechunit_name: str = "ROB_1") -> RWSResult:
        """Robot base frame, as a JSON string."""
        return self.get_generic_json(
            f"/rw/motionsystem/mechunits/{mechunit_name}/baseframe"
        )

    def get_robot_cartesian(self, mechunit_name: str = "ROB_1") -> RWSResult:
        """Returns (The robot cartesian position as JSON string, http_status_code)."""
        return self.get_generic_json(
            f"/rw/motionsystem/mechunits/{mechunit_name}/cartesian"
        )

    def get_robot_robtarget(self, mechunit_name: str = "ROB_1") -> RWSResult:
        """Returns (The robot robtarget position as JSON string, http_status_code)."""
        return self.get_generic_json(
            f"/rw/motionsystem/mechunits/{mechunit_name}/robtarget"
        )

    def get_robot_jointtarget(self, mechunit_name: str = "ROB_1") -> RWSResult:
        """Robot joint target position, as a JSON string."""
        return self.get_generic_json(
            f"/rw/motionsystem/mechunits/{mechunit_name}/jointtarget"
        )

    def get_robot_joint_positions(
        self, mechunit_name: str = "ROB_1", num_ax: int = 6
    ) -> RWSResult:
        """Returns (The robot joint positions as JSON string, http_status_code)."""
        result = self.get_generic_json(
            f"/rw/motionsystem/mechunits/{mechunit_name}/jointtarget"
        )
        # On failure the message is the reason, not JSON.
        if not result.ok:
            return result

        status = result.status
        state = json.loads(result.message)
        if not state:
            return RWSResult(
                f"ERR - {mechunit_name} returned no jointtarget", NO_STATUS
            )

        # extract rax_1 to rax_6 from json
        data = state[0]
        joint_positions = {}
        for i in range(1, num_ax + 1):
            joint_key = f"rax_{i}"
            joint_positions[joint_key] = data.get(joint_key, None)
        result_json = json.dumps(joint_positions, indent=2)
        return RWSResult(result_json, status)

    def get_rapid_execution_state(self) -> RWSResult:
        """Current RAPID execution state, as a JSON string."""
        return self.get_generic_json("/rw/rapid/execution")

    def get_io_networks(self) -> RWSResult:
        """Returns (The IO networks of the robot as JSON string, http_status_code)."""
        return self.get_embed_json("/rw/iosystem/networks")

    def get_io_signals(self) -> RWSResult:
        """Returns (The IO signals of the robot as JSON string, http_status_code)."""
        return self.get_embed_json("/rw/iosystem/signals")

    def get_rapid_tasks(self) -> RWSResult:
        """Returns (The list of RAPID tasks as JSON string, http_status_code)."""
        return self.get_embed_json("/rw/rapid/tasks")

    def get_task_robtarget(self, task_name: str = "T_ROB1") -> RWSResult:
        """Robtarget of a RAPID task, as a JSON string."""
        return self.get_generic_json(f"/rw/rapid/tasks/{task_name}/motion/robtarget")

    def get_task_jointtarget(self, task_name: str = "T_ROB1") -> RWSResult:
        """Jointtarget of a RAPID task, as a JSON string."""
        return self.get_generic_json(f"/rw/rapid/tasks/{task_name}/motion/jointtarget")

    def get_task_modules(self, task_name: str = "T_ROB1") -> RWSResult:
        """Modules in a RAPID task, as a JSON string."""
        return self.get_generic_json(f"/rw/rapid/tasks/{task_name}/modules")

    def get_io_signal(
        self, signal_name: str, network: str = "", device: str = ""
    ) -> RWSResult:
        """State of one IO signal, as a JSON string.

        Most signals resolve without a network and device."""
        if not signal_name:
            raise ValueError("Signal name cannot be empty")
        if network:
            network = network if network.endswith("/") else network + "/"
        if device:
            device = device if device.endswith("/") else device + "/"

        return self.get_embed_json(
            f"/rw/iosystem/signals/{network}{device}{signal_name}"
        )

    def get_rapid_symbol(
        self, symbol_name: str, module_name: str, task_name: str = "T_ROB1"
    ) -> RWSResult:
        """Value of a RAPID symbol in the given module."""
        if not symbol_name or not module_name:
            raise ValueError("Symbol name and module name cannot be empty")

        symbol_url = f"RAPID%2F{task_name}%2F{module_name}%2F{symbol_name}"
        return self.get_generic(f"/rw/rapid/symbol/{symbol_url}/data", "value")

    def get_rapid_symbol_properties(
        self, symbol_name: str, module_name: str, task_name: str = "T_ROB1"
    ) -> RWSResult:
        """Properties of a RAPID symbol, as a JSON string."""
        if not symbol_name or not module_name:
            raise ValueError("Symbol name and module name cannot be empty")
        symbol_url = f"RAPID%2F{task_name}%2F{module_name}%2F{symbol_name}"
        return self.get_embed_json(f"/rw/rapid/symbol/{symbol_url}/properties")

    def get_dipc_queues(self) -> RWSResult:
        """Information about the DIPC queues, as a JSON string."""
        return self.get_embed_json("/rw/dipc")

    def get_dipc_queue_info(self, queue_name: str = "RMQ_T_ROB1") -> RWSResult:
        """Information about one DIPC queue, as a JSON string."""
        return self.get_embed_json(f"/rw/dipc/{queue_name}/information")

    def read_dipc_message(
        self, queue_name: str = "RMQ_T_ROB1", timeout: int = 0
    ) -> tuple[Any | None, int]:
        """Read a message from the DIPC queue."""

        if not queue_name:
            raise ValueError("Queue name cannot be empty")
        if not isinstance(timeout, int) or timeout < 0:
            raise ValueError("Timeout must be a non-negative integer")

        (data, status) = self.get_request(f"/rw/dipc/{queue_name}?timeout={timeout}")

        if status != 200:
            self.logger.error(f"Failed to read message from DIPC queue {queue_name}")

        return (data, status)

    def get_mastership_state(self, domain: str) -> RWSResult:
        """Mastership state of the 'edit' or 'motion' domain."""

        if domain not in ["edit", "motion"]:
            raise ValueError("Invalid domain specified for mastership check")

        return self.get_generic_json(f"/rw/mastership/{domain}")

    def get_rapid_idle(self) -> RWSResult:
        """Returns True if the RAPID execution is idle, False otherwise"""

        if not self.is_running():
            return RWSResult("False", 200)

        (data, status) = self.get_rapid_symbol(
            RobotControllerConstants.Symbols.CURRENT_STATE,
            RobotControllerConstants.Modules.MAIN,
        )
        if status != 200:
            self.logger.error("Failed to get RAPID symbol for current state")
            raise RuntimeError("Failed to get RAPID symbol for current state")
        else:
            if int(data) == 0:  # RAPID symbol for idle state is 0
                return RWSResult("True", 200)
            else:
                return RWSResult("False", 200)

    def motors_on(self) -> RWSResult:
        """Turn on the robot motors."""
        status = self.post_request(
            "/rw/panel/ctrl-state", dataIn={"ctrl-state": "motoron"}
        )
        if status == 204:
            return RWSResult("OK", status)
        else:
            self.logger.error("Failed to turn on motors")
            return RWSResult("ERR - Failed to turn on motors", status)

    def motors_off(self) -> RWSResult:
        """Turn off the robot motors."""
        status = self.post_request(
            "/rw/panel/ctrl-state", dataIn={"ctrl-state": "motoroff"}
        )
        if status == 204:
            return RWSResult("OK", status)
        else:
            self.logger.error("Failed to turn off motors")
            return RWSResult("ERR - Failed to turn off motors", status)

    def restart_controller(self) -> RWSResult:
        """Restart the robot controller. Requires mastership on both domains."""
        if not (self.is_master("edit") and self.is_master("motion")):
            return RWSResult("ERR - Mastership on both domains is required", NO_STATUS)

        status = self.post_request(
            "/rw/panel/restart", dataIn={"restart-mode": "restart"}
        )
        if status == 204:
            return RWSResult("OK", status)
        else:
            self.logger.error("Failed to restart controller")
            return RWSResult("ERR - Failed to restart controller", status)

    def reset_pp(self) -> RWSResult:
        """Reset the program pointer. Requires edit mastership."""
        if not self.is_master("edit"):
            self.logger.error(
                "Mastership on edit domain is required to reset program pointer"
            )
            return RWSResult(
                "ERR - Mastership on edit domain is required to reset program pointer",
                -1,
            )

        status = self.post_request("/rw/rapid/execution/resetpp")
        if status == 204:
            return RWSResult("OK", status)
        else:
            self.logger.error("Failed to reset program pointer")
            return RWSResult("ERR - Failed to reset program pointer", status)

    def start_rapid_script(self) -> RWSResult:
        """Start RAPID execution. Needs edit mastership; motion mastership fails."""

        if not self.is_master("edit") or self.is_master("motion"):
            self.logger.error(
                "Mastership on edit domain is required to start rapid script. Mastership on motion domain will result in this error"
            )
            return RWSResult(
                "ERR - Mastership on edit domain is required to start rapid script. Mastership on motion domain will result in this error",
                -1,
            )

        status = self.post_request(
            "/rw/rapid/execution/start",
            dataIn={
                "regain": "continue",
                "execmode": "continue",
                "cycle": "once",
                "condition": "none",
                "stopatbp": "disabled",
                "alltaskbytsp": "false",
            },
        )
        if status == 204:
            return RWSResult("OK", status)
        else:
            self.logger.error(
                "Failed to start rapid script, ensure motors are on and controller is in automatic mode"
            )
            return RWSResult(
                "ERR - Failed to start rapid script, ensure motors are on and controller is in automatic mode",
                status,
            )

    def stop_rapid_script(self) -> RWSResult:
        """Stop RAPID execution."""
        status = self.post_request(
            "/rw/rapid/execution/stop", dataIn={"stopmode": "stop", "usetsp": "normal"}
        )
        if status == 204:
            return RWSResult("OK", status)
        else:
            self.logger.error("Failed to stop rapid script")
            return RWSResult("ERR - Failed to stop rapid script", status)

    def request_mastership(self, domain: str | None = None) -> RWSResult:
        """Request mastership on a domain ('edit'/'motion') or all if None."""

        if domain not in [None, "edit", "motion"]:
            raise ValueError("Invalid domain specified for mastership request")

        if domain:
            status = self.post_request(f"/rw/mastership/{domain}/request")
            if status != 204:
                self.logger.error(f"Failed to obtain mastership on {domain} domain")
                return RWSResult(
                    f"ERR - Failed to obtain mastership on {domain}", status
                )
            return RWSResult("OK", status)

        else:
            status = self.post_request("/rw/mastership/request")
            if status != 204:
                self.logger.error("Failed to obtain mastership on all domains")
                return RWSResult(
                    "ERR - Failed to obtain mastership on all domains", status
                )
            return RWSResult("OK", status)

    def release_mastership(self, domain: str | None = None) -> RWSResult:
        """Release mastership on a domain ('edit'/'motion') or all if None."""

        if domain not in [None, "edit", "motion"]:
            raise ValueError("Invalid domain specified for mastership release")

        if domain:
            status = self.post_request(f"/rw/mastership/{domain}/release")
            return (
                RWSResult("OK", status)
                if status == 204
                else RWSResult(
                    f"ERR - Mastership on {domain} domain was not released", status
                )
            )

        else:
            status = self.post_request("/rw/mastership/release")
            return (
                RWSResult("OK", status)
                if status == 204
                else RWSResult(
                    "ERR - Mastership on all domains was not released", status
                )
            )

    def set_io_signal(
        self, signal_name: str, signal_value: str, network: str = "", device: str = ""
    ) -> RWSResult:
        """Set the value of an I/O signal."""
        if not signal_name or signal_value is None:
            raise ValueError("Signal name and value cannot be empty")
        if network:
            network = network if network.endswith("/") else network + "/"
        if device:
            device = device if device.endswith("/") else device + "/"

        path = f"/rw/iosystem/signals/{network}{device}{signal_name}/set-value"

        status = self.post_request(path, dataIn={"lvalue": signal_value})

        if status == 204:
            return RWSResult("OK", status)
        else:
            self.logger.error(f"Failed to set IO signal {signal_name}")
            return RWSResult(f"ERR - Failed to set IO signal {signal_name}", status)

    def set_speedratio(self, speed_ratio: str) -> RWSResult:
        """Set the robot speed ratio (0-100)."""
        speed_ratio_int = int(speed_ratio)
        if not (0 <= speed_ratio_int <= 100):
            raise ValueError("Speed ratio must be between 0 and 100")

        status = self.post_request(
            "/rw/panel/speedratio", dataIn={"speed-ratio": speed_ratio}
        )

        if status == 204:
            return RWSResult("OK", status)
        else:
            self.logger.error(f"Failed to set speed ratio to {speed_ratio}")
            return RWSResult(
                f"ERR - Failed to set speed ratio to {speed_ratio}", status
            )

    def reset_energy_info(self) -> RWSResult:
        """Reset accumulated energy information."""
        status = self.post_request("/rw/system/energy/reset")

        if status == 204:
            return RWSResult("OK", status)
        else:
            self.logger.error("Failed to reset energy information")
            return RWSResult("ERR - Failed to reset energy information", status)

    def set_rapid_symbol_raw(
        self, value: str, symbol_name: str, module_name: str, task_name: str = "T_ROB1"
    ) -> RWSResult:
        """Set a RAPID symbol value. Requires edit mastership."""
        if not symbol_name or not module_name:
            raise ValueError("Symbol name and module name cannot be empty")

        symbol_url = f"RAPID%2F{task_name}%2F{module_name}%2F{symbol_name}"

        # value_quoted = f'"{value}"'  # RAPID strings are quoted

        status = self.post_request(
            f"/rw/rapid/symbol/{symbol_url}/data", dataIn={"value": value}
        )

        if status == 204:
            return RWSResult("OK", status)
        elif status == 400:
            self.logger.error(
                f"ERR - String type has to be double quoted .e.g '\"{value}\"' "
            )
            return RWSResult(
                f"ERR - String type has to be in format '\"{value}\"' ", status
            )

        else:
            self.logger.error(
                f"Failed to set RAPID symbol {symbol_name} in module {module_name}, ensure mastership on edit domain"
            )
            return RWSResult(
                f"ERR - Failed to set RAPID symbol {symbol_name} in module {module_name}, ensure mastership on edit domain",
                status,
            )

    def create_dipc_queue(
        self, queue_name: str, queue_size: str, message_size: str
    ) -> RWSResult:
        """Create a DIPC queue with the given name and size."""
        if not queue_name:
            raise ValueError("Queue name cannot be empty")
        if not queue_size.isdigit() or int(queue_size) <= 0:
            raise ValueError("Queue size must be a positive integer")
        if not message_size.isdigit() or int(message_size) <= 0:
            raise ValueError("Message size must be a positive integer")

        datastr = f"dipc-queue-name={queue_name}&dipc-queue-size={queue_size}&dipc-max-msg-size={message_size}"

        status = self.post_request("/rw/dipc", dataIn=datastr)

        if status == 201:
            return RWSResult("OK", status)
        else:
            self.logger.error(f"Failed to create DIPC queue {queue_name}")
            return RWSResult(f"ERR - Failed to create DIPC queue {queue_name}", status)

    def send_dipc_message(
        self, message: str, userdef: str = "1", queue_name: str = "RMQ_T_ROB1"
    ) -> RWSResult:
        """Send a message to a DIPC queue."""
        if not queue_name or not message:
            raise ValueError("Queue name and message cannot be empty")
        if not userdef.isdigit() or not (0 <= int(userdef) <= 255):
            raise ValueError("Userdef must be an integer between 0 and 255")

        payload = {
            "dipc-src-queue-name": queue_name,
            "dipc-cmd": 0,
            "dipc-userdef": userdef,
            "dipc-msgtype": 1,
            "dipc-data": message,
        }

        # Before the payload dict above, this went out as one urlencoded
        # string: dipc-src-queue-name=..&dipc-cmd=111&dipc-userdef=222

        status = self.post_request(
            f"/rw/dipc/{queue_name}/?action=dipc-send", dataIn=payload
        )

        if status == 204:
            return RWSResult("OK", status)
        else:
            # Not logged: a full queue is ordinary back-pressure, and the
            # caller already gets the reason in the returned message.
            return RWSResult(
                f"Failed to send message to DIPC queue (QUEUE PROBABLY FULL){queue_name}",
                status,
            )

    # GET METHODS - non-string return types, for internal Python use
    def is_running(self) -> bool:
        """Returns True if the RAPID execution is running, False otherwise"""
        (data, status) = self.get_rapid_execution_state()
        if status != 200:
            self.logger.error("Failed to get RAPID execution state")
            raise RuntimeError("Failed to get RAPID execution state")

        state = json.loads(data)[0]["ctrlexecstate"]

        if state == "running":
            return True
        elif state == "stopped":
            return False
        else:
            raise RuntimeError(f"Unknown RAPID execution state: {state}")

    def is_rapid_idle(self) -> bool:
        """Returns True if the RAPID execution is idle, False otherwise"""

        if not self.is_running():
            return False

        (data, status) = self.get_rapid_symbol(
            RobotControllerConstants.Symbols.CURRENT_STATE,
            RobotControllerConstants.Modules.MAIN,
        )
        if status != 200:
            self.logger.error("Failed to get RAPID symbol for current state")
            raise RuntimeError("Failed to get RAPID symbol for current state")
        else:
            return int(data) == 0

    def is_master(self, domain: str | None = None) -> bool:
        """Whether we hold mastership on the 'edit' or 'motion' domain."""
        if domain not in ["edit", "motion"]:
            raise ValueError("Invalid domain specified")

        (data, status) = self.get_request(f"/rw/mastership/{domain}")
        if status != 200 or data is None:
            raise RuntimeError(
                f"Could not read mastership of the {domain} domain (status={status})"
            )

        state = data.get("state") or []
        if not state:
            raise RuntimeError(f"Mastership response for {domain} carried no state")

        # RWS answers in strings, so "false" would be truthy on its own.
        held = str(state[0].get("mastershipheldbyme", "false")).strip().lower()
        return held in ("true", "1")

    def make_robot_ready(self) -> RWSResult:
        """Prepare the robot for operation (mastership, motors, RAPID start)."""

        if not self.is_master("edit"):
            mess, code = self.request_mastership("edit")
            if code != 204:
                return RWSResult(
                    "ERR - Failed to obtain mastership on edit domain", code
                )

        if self.is_master("motion"):
            mess, code = self.release_mastership("motion")
            if code != 204:
                return RWSResult(
                    "ERR - Failed to release mastership on motion domain", code
                )

        if not self.is_running():
            mess, code = self.get_opmode_state()
            if mess != "auto":
                return RWSResult(
                    f"ERR - Failed controller is in {mess} mode, change to auto mode",
                    code,
                )

            mess, code = self.get_controller_state()

            if mess != "motoron":
                _, _ = self.motors_on()
                time.sleep(1)

            mess, code = self.get_controller_state()
            if mess != "motoron":
                return RWSResult(f"ERR - Failed to turn on motors: {mess}", code)

            mess, code = self.reset_pp()
            if code != 204:
                return RWSResult(f"ERR - Failed to reset program pointer: {mess}", code)

            time.sleep(1)
            mess, code = self.get_rapid_execution_state()
            state = json.loads(mess)[0]["ctrlexecstate"]
            self.logger.info(f"RAPID execution state: {state}")
            if state != "running":
                mess, code = self.start_rapid_script()
                if mess != "OK":
                    return RWSResult(
                        f"ERR - Failed to start RAPID script: {mess}", code
                    )
                time.sleep(1)

            mess, code = self.get_rapid_execution_state()
            state = json.loads(mess)[0]["ctrlexecstate"]
            if state == "running":
                return RWSResult("Robot set up correctly", code)
            else:
                return RWSResult(f"ERR - Failed to start RAPID script: {mess}", code)

        else:
            return RWSResult("Robot is already running", 200)

    def apply_rapid_writes(
        self, writes: Sequence[tuple[str, str, str, float]]
    ) -> RWSResult:
        """Set a series of RAPID symbols, stopping at the first failure.

        Entries are (value, symbol, module, settle_s). Reporting OK after a
        failed write would leave the robot on the previous routine.
        """
        for value, symbol, module, settle_s in writes:
            write = self.set_rapid_symbol_raw(value, symbol, module)
            if not write.ok:
                self.logger.error(
                    f"Failed to set RAPID symbol {symbol}: {write.message}"
                )
                return RWSResult(
                    f"ERR - Failed to set RAPID symbol {symbol}", write.status
                )
            time.sleep(settle_s)

        return RWSResult("OK", 200)

    def run_move_command(
        self, motion_command: str, robtarget: str, speed: str
    ) -> RWSResult:
        """Execute a motion command (MoveL/MoveJ) to the given robtarget."""
        if self.is_rapid_idle():
            if motion_command not in [
                RobotControllerConstants.MotionCommands.MOVE_L,
                RobotControllerConstants.MotionCommands.MOVE_J,
            ]:
                raise ValueError(f"Unsupported motion command: {motion_command}")
            if motion_command == RobotControllerConstants.MotionCommands.MOVE_L:
                routine_name = RobotControllerConstants.Routines.SINGLE_MOVE_L
            else:
                routine_name = RobotControllerConstants.Routines.SINGLE_MOVE_J

            writes = [
                (
                    f'"{routine_name}"',
                    RobotControllerConstants.Symbols.ROUTINE_NAME,
                    RobotControllerConstants.Modules.RAPID,
                    0.1,
                ),
                (
                    speed,
                    RobotControllerConstants.Symbols.SPEED,
                    RobotControllerConstants.Modules.USER,
                    0.0,
                ),
                (
                    robtarget,
                    RobotControllerConstants.Symbols.RECEIVED_ROBTARGET,
                    RobotControllerConstants.Modules.USER,
                    0.2,
                ),
                (
                    RobotControllerConstants.States.EXECUTE,
                    RobotControllerConstants.Symbols.CURRENT_STATE,
                    RobotControllerConstants.Modules.MAIN,
                    0.0,
                ),
            ]

            return self.apply_rapid_writes(writes)

        else:
            return RWSResult(
                "ERR - Cannot execute motion command while RAPID is not idle", NO_STATUS
            )

    def run_rapid_routine(self, routine_name: str) -> RWSResult:
        """Run a RAPID routine on the robot."""
        if self.is_rapid_idle():
            writes = [
                (
                    f'"{routine_name}"',
                    RobotControllerConstants.Symbols.ROUTINE_NAME,
                    RobotControllerConstants.Modules.RAPID,
                    0.2,
                ),
                (
                    RobotControllerConstants.States.EXECUTE,
                    RobotControllerConstants.Symbols.CURRENT_STATE,
                    RobotControllerConstants.Modules.MAIN,
                    0.0,
                ),
            ]

            return self.apply_rapid_writes(writes)

        else:
            return RWSResult(
                "ERR - Cannot execute RAPID routine while RAPID is not idle", NO_STATUS
            )
