
import time
from robot_control.exceptions import RWSConnectionError, RWSError, RWSStateError
from robot_control.constants import MotionCommands, RapidConfig, RoutineNames

import json
from robot_control.rws.provider import RWSClient


class RWSInterface(RWSClient):
    """High-level interface for ABB Robot Web Services (RWS).
    Wraps HTTP requests to the robot controller for state, RAPID, IO, and DIPC operations.

    The RAPID-side vocabulary - task, mechanical unit, DIPC queue, module and
    symbol names - comes from `rapid`, so the same driver can talk to a program
    that names things differently.

    Failures raise: RWSConnectionError when the request never got through,
    RWSStateError when the controller answered and refused, ValueError for a bad
    argument. Methods that return a value return the value, not a status - the
    exception is the status. The one deliberate exception is send_dipc_message,
    where a full queue is an answer rather than a failure."""


    def __init__(self, host: str, username: str, password: str, port: int = 80, logger=None,
                 timeout_s: float = 2.0, rapid: RapidConfig = None,
                 routines: RoutineNames = None):

        super().__init__(host, username, password, port=port, logger=logger,
                         timeout_s=timeout_s)
        self.rapid = rapid if rapid is not None else RapidConfig()
        self.routines = routines if routines is not None else RoutineNames()

    
    def _command(self, path: str, data=None, expect: int = 204, what: str = "") -> None:
        """POST something and raise unless the controller accepted it.

        Every command below is this: send, check one status, complain in the
        same words. Returning nothing is the point - there is no failure to
        forget to check.
        """
        status = self.post_request(path, dataIn=data)
        if status != expect:
            raise RWSStateError(f"{what} refused by the controller", status)

    def get_generic(self, endpoint: str, feature: str) -> tuple[str, int]:
        (data, status) = self.get_request(endpoint)
        if data is None:
            raise RWSError(f"GET {endpoint} returned no body", status)

        data = data['state'][0]
        return (data.get(feature, 'unknown').lower(), status)
    
    def get_embed_json(self, endpoint: str) -> tuple[str, int]:
        (data, status) = self.get_request(endpoint)
        if data is None:
            raise RWSError(f"GET {endpoint} returned no body", status)

        data = data.get('_embedded', {}).get('resources', [])
        data_json = json.dumps(data, indent=2)
        return (data_json, status)
    
    def get_generic_json(self, endpoint: str) -> tuple[str, int]:
        (data, status) = self.get_request(endpoint)
        if data is None:
            raise RWSError(f"GET {endpoint} returned no body", status)

        data = data['state']
        data_json = json.dumps(data, indent=2)
        return (data_json, status)
    



    def get_clock(self) -> tuple[str, int]:
        """Returns (A string representing the current clock value from the robot controller, http_status_code)."""
        return self.get_generic("/ctrl/clock", "datetime")

    def get_controller_state(self) -> tuple[str, int]:
        """Returns (The state of the controller (motors on/off, guardstop, emergencystop, init), http_status_code)."""
        return self.get_generic("/rw/panel/ctrl-state", "ctrlstate")
        
    def get_opmode_state(self) -> tuple[str, int]:
        """Returns (The current operational mode of the robot (auto, man, manf), http_status_code)."""
        return self.get_generic("/rw/panel/opmode", "opmode")

    def get_safety_mode(self) -> tuple[str, int]:
        """Returns (The current safety mode of the robot, http_status_code)."""
        return self.get_generic("/ctrl/safety/mode", "safetymode")
    
        
    def get_rapid_retcode(self, retcode_name) -> tuple[str, int]:
        """Returns (RAPID return code value, http_status_code)."""
        if not retcode_name:
            raise ValueError("Return code name cannot be empty")
        return self.get_request(f"/rw/retcode/?code={retcode_name}")
    
    def get_user_uas(self) -> tuple[str, int]:
        """Returns (The user-defined UAS variables as JSON string, http_status_code)."""
        return self.get_request("/uas/user/grants")
    
    def get_all_grants(self) -> tuple[str, int]:
        """Returns (The user-defined UAS variables as JSON string, http_status_code)."""
        return self.get_request("/uas/grants")
    

    def get_speedratio(self) -> tuple[str, int]:
        """Returns (The current speed ratio of the robot, http_status_code)."""
        return self.get_generic("/rw/panel/speedratio", "speedratio")

    def get_robot_type(self) -> tuple[str, int]:
        """Returns (The type of the robot (e.g., CRB 15000-10/1.52), http_status_code)."""
        return self.get_generic("/rw/system/robottype", "robot-type")
    
    def get_system_options(self) -> tuple[str, int]:
        """Returns (The system options of the robot as JSON string, http_status_code)."""
        return self.get_generic_json("/rw/system/options")
    
    def get_system_products(self) -> tuple[str, int]:
        """Returns (The system products of the robot as JSON string, http_status_code)."""
        return self.get_generic_json("/rw/system/products")
    
    def get_energy_info(self) -> tuple[str, int]:
        """Returns (The energy information of the robot as JSON string, http_status_code)."""
        return self.get_generic_json("/rw/system/energy")
    
    def get_leadthrough_state(self) -> tuple[str, int]:
        """Returns (The leadthrough state of the robot, http_status_code)."""
        return self.get_generic(f"/rw/motionsystem/mechunits/{self.rapid.mechunit}/lead-through", "status")
    
    def get_robot_baseframe(self) -> tuple[str, int]:
        """Returns (The robot base frame information as JSON string, http_status_code)."""
        return self.get_generic_json(f"/rw/motionsystem/mechunits/{self.rapid.mechunit}/baseframe")
    
    def get_robot_cartesian(self) -> tuple[str, int]:
        """Returns (The robot cartesian position as JSON string, http_status_code)."""
        return self.get_generic_json(f"/rw/motionsystem/mechunits/{self.rapid.mechunit}/cartesian")

    def get_robot_robtarget(self) -> tuple[str, int]:
        """Returns (The robot robtarget position as JSON string, http_status_code)."""
        return self.get_generic_json(f"/rw/motionsystem/mechunits/{self.rapid.mechunit}/robtarget")

    def get_robot_jointtarget(self) -> tuple[str, int]:
        """Returns (The robot joint target position as JSON string, http_status_code)."""
        return self.get_generic_json(f"/rw/motionsystem/mechunits/{self.rapid.mechunit}/jointtarget")
    
    def get_robot_joint_positions(self) -> tuple[str, int]:
        """Returns (The robot joint positions as JSON string, http_status_code)."""
        result_json, status = self.get_generic_json(f"/rw/motionsystem/mechunits/{self.rapid.mechunit}/jointtarget")
        #extract rax_1 to rax_6 from json
        data = json.loads(result_json)[0]
        joint_positions = {}
        for i in range(1, self.rapid.num_axes + 1):
            joint_key = f"rax_{i}"
            joint_positions[joint_key] = data.get(joint_key, None)
        result_json = json.dumps(joint_positions, indent=2)
        return result_json, status
    
    def get_rapid_execution_state(self) -> tuple[str, int]:
        """Returns (The current state of the RAPID execution as JSON string, http_status_code)."""
        return self.get_generic_json("/rw/rapid/execution")

    def get_io_networks(self) -> tuple[str, int]:
        """Returns (The IO networks of the robot as JSON string, http_status_code)."""
        return self.get_embed_json("/rw/iosystem/networks")
    
    def get_io_signals(self) -> tuple[str, int]:
        """Returns (The IO signals of the robot as JSON string, http_status_code)."""
        return self.get_embed_json("/rw/iosystem/signals")

    def get_rapid_tasks(self) -> tuple[str, int]:
        """Returns (The list of RAPID tasks as JSON string, http_status_code)."""
        return self.get_embed_json("/rw/rapid/tasks")

    def get_task_robtarget(self) -> tuple[str, int]:
        """Returns (The robtarget of the configured RAPID task as JSON string, http_status_code)."""
        return self.get_generic_json(f"/rw/rapid/tasks/{self.rapid.task}/motion/robtarget")
    
    def get_task_jointtarget(self) -> tuple[str, int]:
        """Returns (The jointtarget of the configured RAPID task as JSON string, http_status_code)."""
        return self.get_generic_json(f"/rw/rapid/tasks/{self.rapid.task}/motion/jointtarget")
    
    def get_task_modules(self) -> tuple[str, int]:
        """Returns (The modules of the configured RAPID task as JSON string, http_status_code)."""
        return self.get_generic_json(f"/rw/rapid/tasks/{self.rapid.task}/modules")

    def get_io_signal(self, signal_name, network="", device="") -> tuple[str, int]:
        """Returns (The IO signal state for a specific signal as JSON string, http_status_code). signal_name: Name of the IO signal.
            network: Network name (optional, default: "").
            device: Device name (optional, default: "").
        Note:
            For some signals, network and device do not have to be specified."""
        if not signal_name:
            raise ValueError("Signal name cannot be empty")
        if network:
            network = network if network.endswith('/') else network + '/'
        if device:
            device = device if device.endswith('/') else device + '/'

        return self.get_embed_json(f"/rw/iosystem/signals/{network}{device}{signal_name}")
    
    def get_rapid_symbol(self, symbol_name, module_name):
        """Returns (The value of a specific RAPID symbol as string, http_status_code). symbol_name: Name of the RAPID symbol.
            module_name: Name of the module containing the symbol."""
        if not symbol_name or not module_name:
            raise ValueError("Symbol name and module name cannot be empty")

        symbol_url = f"RAPID%2F{self.rapid.task}%2F{module_name}%2F{symbol_name}"
        return self.get_generic(f"/rw/rapid/symbol/{symbol_url}/data", "value")

    def get_rapid_symbol_properties(self, symbol_name, module_name):
        """Returns (The properties of a specific RAPID symbol as JSON string, http_status_code). symbol_name: Name of the RAPID symbol.
            module_name: Name of the module containing the symbol."""
        if not symbol_name or not module_name:
            raise ValueError("Symbol name and module name cannot be empty")
        symbol_url = f"RAPID%2F{self.rapid.task}%2F{module_name}%2F{symbol_name}"
        return self.get_embed_json(f"/rw/rapid/symbol/{symbol_url}/properties")
    
    
    def get_dipc_queues(self):
        """Returns (The information about the DIPC queues as JSON string, http_status_code)."""
        return self.get_embed_json("/rw/dipc")
  
    
    def get_dipc_queue_info(self):
        """Returns (The information about the configured DIPC queue as JSON string, http_status_code)."""
        return self.get_embed_json(f"/rw/dipc/{self.rapid.dipc_queue}/information")
    
    def read_dipc_message(self, timeout=0):
        """Read a message from the specified DIPC queue. Returns (data, http_status_code)."""

        if not isinstance(timeout, int) or timeout < 0:
            raise ValueError("Timeout must be a non-negative integer")
        
        (data, status) = self.get_request(f"/rw/dipc/{self.rapid.dipc_queue}?timeout={timeout}")

        if status != 200:
            self.logger.error(f"Failed to read message from DIPC queue {self.rapid.dipc_queue}")

        return (data, status)

    def get_mastership_state(self, domain: str) -> tuple[str, int]:
        """Check if the client holds mastership on the given domain ('edit' or 'motion')."""

        if domain not in ['edit', 'motion']:
            raise ValueError(f"Unknown mastership domain {domain!r}")
        return self.get_generic_json(f"/rw/mastership/{domain}")


    def motors_on(self) -> None:
        """Turn on the robot motors."""
        self._command("/rw/panel/ctrl-state", {'ctrl-state': 'motoron'}, what="motors on")

    def motors_off(self) -> None:
        """Turn off the robot motors."""
        self._command("/rw/panel/ctrl-state", {'ctrl-state': 'motoroff'}, what="motors off")

    def restart_controller(self) -> None:
        """Restart the robot controller. Requires mastership on both domains."""
        if not (self.is_master('edit') and self.is_master('motion')):
            raise RWSStateError("Restart needs mastership on both domains")
        self._command("/rw/panel/restart", {'restart-mode': 'restart'}, what="controller restart")

    def reset_pp(self) -> None:
        """Reset the program pointer. Requires edit mastership."""
        if not self.is_master('edit'):
            raise RWSStateError("Resetting the program pointer needs mastership on edit")
        self._command("/rw/rapid/execution/resetpp", what="program pointer reset")

    def start_rapid_script(self) -> None:
        """Start RAPID execution. Needs mastership on edit and NOT on motion."""
        if not self.is_master('edit') or self.is_master('motion'):
            raise RWSStateError(
                "Starting the program needs mastership on edit and none on motion")
        self._command(
            "/rw/rapid/execution/start",
            {'regain': 'continue', 'execmode': 'continue', 'cycle': 'once',
             'condition': 'none', 'stopatbp': 'disabled', 'alltaskbytsp': 'false'},
            what="program start (check motors are on and the controller is in auto)")

    def stop_rapid_script(self) -> None:
        """Stop RAPID execution."""
        self._command("/rw/rapid/execution/stop", {'stopmode': 'stop', 'usetsp': 'normal'},
                      what="program stop")

    @staticmethod
    def _mastership_path(domain, action: str) -> str:
        if domain not in (None, 'edit', 'motion'):
            raise ValueError(f"Unknown mastership domain {domain!r}, expected 'edit' or 'motion'")
        return f"/rw/mastership/{domain}/{action}" if domain else f"/rw/mastership/{action}"

    def request_mastership(self, domain=None) -> None:
        """Take mastership of a domain ('edit'/'motion'), or of all if None."""
        self._command(self._mastership_path(domain, "request"),
                      what=f"mastership request on {domain or 'all domains'}")

    def release_mastership(self, domain=None) -> None:
        """Give mastership of a domain ('edit'/'motion') back, or of all if None."""
        self._command(self._mastership_path(domain, "release"),
                      what=f"mastership release on {domain or 'all domains'}")

    def set_io_signal(self, signal_name: str, signal_value: str, network="", device="") -> None:
        """Set the value of an I/O signal."""
        if not signal_name or signal_value is None:
            raise ValueError("Signal name and value are both required")
        if network:
            network = network if network.endswith('/') else network + '/'
        if device:
            device = device if device.endswith('/') else device + '/'


        self._command(f"/rw/iosystem/signals/{network}{device}{signal_name}/set-value",
                      {'lvalue': signal_value}, what=f"setting IO signal {signal_name}")
        

    def set_speedratio(self, speed_ratio: str) -> None:
        """Set the robot speed ratio (0-100)."""
        if not 0 <= int(speed_ratio) <= 100:
            raise ValueError(f"Speed ratio {speed_ratio} is outside 0-100")
        self._command("/rw/panel/speedratio", {'speed-ratio': speed_ratio},
                      what=f"speed ratio {speed_ratio}")

    def reset_energy_info(self) -> None:
        """Reset accumulated energy information."""
        self._command("/rw/system/energy/reset", what="energy info reset")

    def set_rapid_symbol_raw(self, value: str, symbol_name: str, module_name: str) -> None:
        """Set a RAPID symbol value. Requires edit mastership."""
        if not symbol_name or not module_name:
            raise ValueError("Symbol name and module name are both required")

        symbol_url = f"RAPID%2F{self.rapid.task}%2F{module_name}%2F{symbol_name}"
        status = self.post_request(f"/rw/rapid/symbol/{symbol_url}/data", dataIn={'value': value})

        if status == 204:
            return
        if status == 400:
            # The usual cause: a RAPID string has to arrive already quoted.
            raise RWSStateError(
                f'Controller rejected {value!r} for {symbol_name}; '
                f'a string value has to be quoted, as \'"{value}"\'', status)
        raise RWSStateError(
            f"Could not set {symbol_name} in {module_name} - check mastership on edit",
            status)



    def create_dipc_queue(self, queue_name: str, queue_size: str, message_size: str) -> None:
        """Create a DIPC queue with the given name and size."""
        if not queue_name:
            raise ValueError("Queue name is required")
        if not queue_size.isdigit() or int(queue_size) <= 0:
            raise ValueError(f"Queue size {queue_size!r} must be a positive integer")
        if not message_size.isdigit() or int(message_size) <= 0:
            raise ValueError(f"Message size {message_size!r} must be a positive integer")

        datastr = f'dipc-queue-name={queue_name}&dipc-queue-size={queue_size}&dipc-max-msg-size={message_size}'
        # print(datastr)

        self._command("/rw/dipc", datastr, expect=201, what=f"creating DIPC queue {queue_name}")

    def send_dipc_message(self, message: str, userdef: str = "1") -> bool:
        """Put one message on the DIPC queue.

        Returns True if the controller took it, False if the queue is full -
        that is normal back-pressure while the robot works through the path, and
        the caller is expected to wait and try again. Anything else raises.
        """
        if not message:
            raise ValueError("Message is required")
        if not userdef.isdigit() or not (0 <= int(userdef) <= 255):
            raise ValueError(f"userdef {userdef!r} must be an integer between 0 and 255")

        payload = {"dipc-src-queue-name": self.rapid.dipc_queue, "dipc-cmd": 0,
                   "dipc-userdef": userdef, "dipc-msgtype": 1, "dipc-data": message}
        status = self.post_request(f"/rw/dipc/{self.rapid.dipc_queue}/?action=dipc-send",
                                   dataIn=payload)

        if status == 204:
            return True
        if status == 500:
            return False
        raise RWSStateError(f"DIPC queue {self.rapid.dipc_queue} rejected the message", status)



    # GET METHODS - non-string return types, for internal Python use
    def is_running(self):
        """ Returns True if the RAPID execution is running, False otherwise """
        (data, status) = self.get_rapid_execution_state()
        if status != 200:
            self.logger.error("Failed to get RAPID execution state")
            raise RWSError("Could not read the RAPID execution state", status)

        state = json.loads(data)[0]["ctrlexecstate"]

        if state == 'running':
            return True
        elif state == 'stopped':
            return False
        else:
            raise RWSError(f"Unknown RAPID execution state: {state}")
        
    def is_rapid_idle(self):
        """ Returns True if the RAPID execution is idle, False otherwise """

        if not self.is_running():
            return False

        (data, status) = self.get_rapid_symbol(self.rapid.symbol_current_state, self.rapid.module_main)
        if status != 200:
            self.logger.error("Failed to get RAPID symbol for current state")
            raise RWSError("Could not read the current state symbol", status)
        else:

            return str(data).strip() == self.rapid.state_idle
    
    def is_master(self, domain=None): 
        """ 
        Returns True if the client has mastership on given domain (Edit or Motion), False otherwise 
        """
        if domain not in ['edit', 'motion']:
            raise ValueError("Invalid domain specified")
        
        if domain:
            (data,status) = self.get_request(f"/rw/mastership/{domain}")
            mastership  = data.get('state', [])[0]
            held_by_me = mastership.get('mastershipheldbyme', False)
            if held_by_me:
                return True
            else:
                return False
              
    def make_robot_ready(self) -> str:
        """Get the controller to the point where it will accept motion.

        Mastership on edit, none on motion, motors on, program pointer at the
        top, program running. Each step raises if it does not take, so getting
        to the end means the robot is ready.
        """
        if self.is_running():
            return "Robot is already running"

        if not self.is_master('edit'):
            self.request_mastership('edit')
        if self.is_master('motion'):
            self.release_mastership('motion')

        mode, status = self.get_opmode_state()
        if mode != "auto":
            raise RWSStateError(f"Controller is in {mode} mode, switch it to auto", status)

        state, status = self.get_controller_state()
        if state != "motoron":
            self.motors_on()
            time.sleep(self.rapid.controller_settle_s)
            state, status = self.get_controller_state()
            if state != "motoron":
                raise RWSStateError(f"Motors did not come on, controller is {state}", status)

        self.reset_pp()
        time.sleep(self.rapid.controller_settle_s)

        if not self.is_running():
            self.start_rapid_script()
            time.sleep(self.rapid.controller_settle_s)
            if not self.is_running():
                raise RWSStateError("Program did not start")

        return "Robot set up correctly"

    def run_move_command(self, motion_command: str, robtarget: str, speed: str) -> None:
        """Send the robot to one robtarget with MoveL or MoveJ."""
        single = {MotionCommands.MOVE_L: self.routines.single_move_l,
                  MotionCommands.MOVE_J: self.routines.single_move_j}
        if motion_command not in single:
            raise ValueError(f"Unsupported motion command {motion_command!r}")
        if not self.is_rapid_idle():
            raise RWSStateError("Robot is busy, cannot start a motion command")

        self.set_rapid_symbol_raw(f'"{single[motion_command]}"',
                                  self.rapid.symbol_routine_name, self.rapid.module_rapid)
        time.sleep(self.rapid.symbol_settle_s)
        self.set_rapid_symbol_raw(speed, self.rapid.symbol_speed, self.rapid.module_user)
        self.set_rapid_symbol_raw(robtarget, self.rapid.symbol_received_robtarget,
                                  self.rapid.module_user)
        time.sleep(2 * self.rapid.symbol_settle_s)  # both symbols land before the state flips
        self.set_rapid_symbol_raw(self.rapid.state_execute, self.rapid.symbol_current_state,
                                  self.rapid.module_main)

    def run_rapid_routine(self, routine_name: str) -> None:
        """Run a RAPID routine on the robot."""
        if not self.is_rapid_idle():
            raise RWSStateError(f"Robot is busy, cannot start {routine_name}")

        self.set_rapid_symbol_raw(f'"{routine_name}"',
                                  self.rapid.symbol_routine_name, self.rapid.module_rapid)
        time.sleep(2 * self.rapid.symbol_settle_s)  # the name lands before the state flips
        self.set_rapid_symbol_raw(self.rapid.state_execute, self.rapid.symbol_current_state,
                                  self.rapid.module_main)


    @staticmethod   
    def pose_to_robtarget(pose) -> str:
        """Convert geometry_msgs/Pose to ABB robtarget string (ROS xyz,w -> ABB w,xyz)."""
        x = pose.position.x
        y = pose.position.y
        z = pose.position.z
        
        # Quaternion: ROS (x,y,z,w) -> ABB (w,x,y,z)
        qw = pose.orientation.w
        qx = pose.orientation.x
        qy = pose.orientation.y
        qz = pose.orientation.z
        
        pos = f"[{x:.3f},{y:.3f},{z:.3f}]"
        orient = f"[{qw:.6f},{qx:.6f},{qy:.6f},{qz:.6f}]"
        confdata = "[0,0,0,0]"
        extax = "[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]"
        
        return f"[{pos},{orient},{confdata},{extax}]"
    
    @staticmethod
    def pose_to_dipc_robtarget(pose) -> str:
        """Convert geometry_msgs/Pose to ABB DIPC robtarget string."""
        # Position (meters -> millimeters)
        x = pose.position.x
        y = pose.position.y
        z = pose.position.z
        
        # Quaternion: ROS (x,y,z,w) -> ABB (w,x,y,z)
        qw = pose.orientation.w
        qx = pose.orientation.x
        qy = pose.orientation.y
        qz = pose.orientation.z
        
        pos = f"[{x:.3f},{y:.3f},{z:.3f}]"
        orient = f"[{qw:.6f},{qx:.6f},{qy:.6f},{qz:.6f}]"
        confdata = "[0,0,0,0]"
        extax = "[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]"
        
        return f"robtarget;[{pos},{orient},{confdata},{extax}]"
    
    @staticmethod
    def joints_to_dipc_jointtarget(joints: list) -> str:
        """Convert a list of 6 joint angles (degrees) to an ABB DIPC jointtarget string.

        Format: jointtarget;[[j1,j2,j3,j4,j5,j6],[9E+09,...]]
        The second array are external axes — all 9E+09 means not used.
        """
        robax = ",".join(f"{j:.4f}" for j in joints[:6])
        extax = "9E+09,9E+09,9E+09,9E+09,9E+09,9E+09"
        return f"jointtarget;[[{robax}],[{extax}]]"

