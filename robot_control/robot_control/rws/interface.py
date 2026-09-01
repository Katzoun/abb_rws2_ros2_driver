
import time
from robot_control.exceptions import RWSException
from robot_control.constants import RobotControllerConstants

import json
from robot_control.rws.provider import RWSClient


class RWSInterface(RWSClient):
    """High-level interface for ABB Robot Web Services (RWS).
    Wraps HTTP requests to the robot controller for state, RAPID, IO, and DIPC operations."""


    def __init__(self, host: str, username: str, password: str, port: int = 80, logger=None):

        super().__init__(host, username, password, port=port, logger=logger)

    
    def get_generic(self, endpoint: str, feature: str) -> tuple[str, int]:
        (data, status) = self.get_request(endpoint)
        if data is None:
            return ("ERR", status)
        
        data = data['state'][0]
        return (data.get(feature, 'unknown').lower(), status)
    
    def get_embed_json(self, endpoint: str) -> tuple[str, int]:
        (data, status) = self.get_request(endpoint)
        if data is None:
            return ("ERR", status)
        
        data = data.get('_embedded', {}).get('resources', [])
        data_json = json.dumps(data, indent=2)
        return (data_json, status)
    
    def get_generic_json(self, endpoint: str) -> tuple[str, int]:
        (data, status) = self.get_request(endpoint)
        if data is None:
            return ("ERR", status)
        
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
            raise RWSException("Return code name cannot be empty")
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
    
    def get_leadthrough_state(self, mechunit_name = "ROB_1") -> tuple[str, int]:
        """Returns (The leadthrough state of the robot, http_status_code)."""
        return self.get_generic(f"/rw/motionsystem/mechunits/{mechunit_name}/lead-through", "status")
    
    def get_robot_baseframe(self, mechunit_name = "ROB_1") -> tuple[str, int]:
        """Returns (The robot base frame information as JSON string, http_status_code)."""
        return self.get_generic_json(f"/rw/motionsystem/mechunits/{mechunit_name}/baseframe")
    
    def get_robot_cartesian(self, mechunit_name = "ROB_1") -> tuple[str, int]:
        """Returns (The robot cartesian position as JSON string, http_status_code)."""
        return self.get_generic_json(f"/rw/motionsystem/mechunits/{mechunit_name}/cartesian")

    def get_robot_robtarget(self, mechunit_name = "ROB_1") -> tuple[str, int]:
        """Returns (The robot robtarget position as JSON string, http_status_code)."""
        return self.get_generic_json(f"/rw/motionsystem/mechunits/{mechunit_name}/robtarget")

    def get_robot_jointtarget(self, mechunit_name = "ROB_1") -> tuple[str, int]:
        """Returns (The robot joint target position as JSON string, http_status_code)."""
        return self.get_generic_json(f"/rw/motionsystem/mechunits/{mechunit_name}/jointtarget")
    
    def get_robot_joint_positions(self, mechunit_name = "ROB_1", num_ax = 6) -> tuple[str, int]:
        """Returns (The robot joint positions as JSON string, http_status_code)."""
        result_json, status = self.get_generic_json(f"/rw/motionsystem/mechunits/{mechunit_name}/jointtarget")
        #extract rax_1 to rax_6 from json
        data = json.loads(result_json)[0]
        joint_positions = {}
        for i in range(1, num_ax + 1):
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

    def get_task_robtarget(self, task_name = "T_ROB1") -> tuple[str, int]:
        """Returns (The robtarget information of a specific RAPID task as JSON string, http_status_code). task_name: Name of the RAPID task (default: "T_ROB1")."""
        return self.get_generic_json(f"/rw/rapid/tasks/{task_name}/motion/robtarget")
    
    def get_task_jointtarget(self, task_name = "T_ROB1") -> tuple[str, int]:
        """Returns (The jointtarget information of a specific RAPID task as JSON string, http_status_code). task_name: Name of the RAPID task (default: "T_ROB1")."""
        return self.get_generic_json(f"/rw/rapid/tasks/{task_name}/motion/jointtarget")
    
    def get_task_modules(self, task_name = "T_ROB1") -> tuple[str, int]:
        """Returns (The list of modules in a specific RAPID task as JSON string, http_status_code). task_name: Name of the RAPID task (default: "T_ROB1")."""
        return self.get_generic_json(f"/rw/rapid/tasks/{task_name}/modules")

    def get_io_signal(self, signal_name, network="", device="") -> tuple[str, int]:
        """Returns (The IO signal state for a specific signal as JSON string, http_status_code). signal_name: Name of the IO signal.
            network: Network name (optional, default: "").
            device: Device name (optional, default: "").
        Note:
            For some signals, network and device do not have to be specified."""
        if not signal_name:
            raise RWSException("Signal name cannot be empty")
        if network:
            network = network if network.endswith('/') else network + '/'
        if device:
            device = device if device.endswith('/') else device + '/'

        return self.get_embed_json(f"/rw/iosystem/signals/{network}{device}{signal_name}")
    
    def get_rapid_symbol(self, symbol_name, module_name, task_name="T_ROB1"):
        """Returns (The value of a specific RAPID symbol as string, http_status_code). symbol_name: Name of the RAPID symbol.
            module_name: Name of the module containing the symbol.
            task_name: Name of the RAPID task (default: "T_ROB1")."""
        if not symbol_name or not module_name:
            raise RWSException("Symbol name and module name cannot be empty")

        symbol_url = f"RAPID%2F{task_name}%2F{module_name}%2F{symbol_name}"
        return self.get_generic(f"/rw/rapid/symbol/{symbol_url}/data", "value")

    def get_rapid_symbol_properties(self, symbol_name, module_name, task_name="T_ROB1"):
        """Returns (The properties of a specific RAPID symbol as JSON string, http_status_code). symbol_name: Name of the RAPID symbol.
            module_name: Name of the module containing the symbol.
            task_name: Name of the RAPID task (default: "T_ROB1")."""
        if not symbol_name or not module_name:
            raise RWSException("Symbol name and module name cannot be empty")
        symbol_url = f"RAPID%2F{task_name}%2F{module_name}%2F{symbol_name}"
        return self.get_embed_json(f"/rw/rapid/symbol/{symbol_url}/properties")
    
    
    def get_dipc_queues(self):
        """Returns (The information about the DIPC queues as JSON string, http_status_code)."""
        return self.get_embed_json("/rw/dipc")
  
    
    def get_dipc_queue_info(self, queue_name="RMQ_T_ROB1"):
        """Returns (The information about a specific DIPC queue as JSON string, http_status_code). queue_name: Name of the DIPC queue (default: "RMQ_T_ROB1")."""
        return self.get_embed_json(f"/rw/dipc/{queue_name}/information")
    
    def read_dipc_message(self, queue_name="RMQ_T_ROB1", timeout=0):
        """Read a message from the specified DIPC queue. Returns (data, http_status_code)."""

        if not queue_name:
            raise RWSException("Queue name cannot be empty")
        if not isinstance(timeout, int) or timeout < 0:
            raise RWSException("Timeout must be a non-negative integer")
        
        (data, status) = self.get_request(f"/rw/dipc/{queue_name}?timeout={timeout}")

        if status != 200:
            self.logger.error(f"Failed to read message from DIPC queue {queue_name}")

        return (data, status)

    def get_mastership_state(self, domain: str) -> tuple[str, int]:
        """Check if the client holds mastership on the given domain ('edit' or 'motion')."""

        if domain not in ['edit', 'motion']:
            self.logger.error("Invalid domain specified for mastership check")
            return ("ERR", -1)

        if domain:
            return self.get_generic_json(f"/rw/mastership/{domain}")


    def motors_on(self) -> tuple[str, int]:
        """Turn on the robot motors."""
        status = self.post_request("/rw/panel/ctrl-state", dataIn={'ctrl-state': 'motoron'})
        if status == 204:
            return ("OK", status)
        else:
            self.logger.error("Failed to turn on motors")
            return ("ERR - Failed to turn on motors", status)

    def motors_off(self) -> tuple[str, int]:
        """Turn off the robot motors."""
        status = self.post_request("/rw/panel/ctrl-state", dataIn={'ctrl-state': 'motoroff'})
        if status == 204:
            return ("OK", status)
        else:
            self.logger.error("Failed to turn off motors")
            return ("ERR - Failed to turn off motors", status)

    def restart_controller(self) -> tuple[str, int]:
        """Restart the robot controller. Requires mastership on both domains."""
        if not (self.is_master('edit') and self.is_master('motion')):
            return ("ERR - Mastership on both domains is required", -1)

        status = self.post_request("/rw/panel/restart", dataIn={'restart-mode': 'restart'})
        if status == 204:
            return ("OK", status)
        else:
            self.logger.error("Failed to restart controller")
            return ("ERR - Failed to restart controller", status)

    def reset_pp(self) -> tuple[str, int]:
        """Reset the program pointer. Requires edit mastership."""
        if not self.is_master('edit'):
            self.logger.error("Mastership on edit domain is required to reset program pointer")
            return ("ERR - Mastership on edit domain is required to reset program pointer", -1)

        status = self.post_request("/rw/rapid/execution/resetpp")
        if status == 204:
            return ("OK", status)
        else:
            self.logger.error("Failed to reset program pointer")
            return ("ERR - Failed to reset program pointer", status)

    def start_rapid_script(self) -> tuple[str, int]:
        """Start RAPID execution. Requires edit mastership only (motion mastership causes error)."""
        
        if not self.is_master('edit') or self.is_master('motion'):
            self.logger.error("Mastership on edit domain is required to start rapid script. Mastership on motion domain will result in this error")
            return ("ERR - Mastership on edit domain is required to start rapid script. Mastership on motion domain will result in this error", -1)

        status = self.post_request("/rw/rapid/execution/start", dataIn={'regain' :'continue' ,'execmode' :'continue' ,'cycle' : 'once' ,'condition' : 'none' ,'stopatbp' : 'disabled','alltaskbytsp' : 'false'})
        if status == 204:
            return ("OK", status)
        else:
            self.logger.error("Failed to start rapid script, ensure motors are on and controller is in automatic mode")
            return ("ERR - Failed to start rapid script, ensure motors are on and controller is in automatic mode", status)

    def stop_rapid_script(self) -> tuple[str, int]:
        """Stop RAPID execution."""
        status = self.post_request("/rw/rapid/execution/stop", dataIn={'stopmode' : 'stop', 'usetsp' : 'normal'})
        if status == 204:
            return ("OK", status)
        else:
            self.logger.error("Failed to stop rapid script")
            return ("ERR - Failed to stop rapid script", status)

    def request_mastership(self, domain=None) -> tuple[str, int]:
        """Request mastership on a domain ('edit'/'motion') or all if None."""

        if domain not in [None, 'edit', 'motion']:
            self.logger.error("Invalid domain specified for mastership request")
            return ("ERR - Invalid domain specified", -1)
        
        if domain:
            status = self.post_request(f"/rw/mastership/{domain}/request")
            if status != 204:
                self.logger.error(f"Failed to obtain mastership on {domain} domain")
                return (f"ERR - Failed to obtain mastership on {domain}", status)
            return ("OK", status)

        else:
            status = self.post_request("/rw/mastership/request")
            if status != 204:
                self.logger.error("Failed to obtain mastership on all domains")
                return ("ERR - Failed to obtain mastership on all domains", status)
            return ("OK", status)

        
    def release_mastership(self, domain=None)  -> tuple[str, int]:
        """Release mastership on a domain ('edit'/'motion') or all if None."""
        
        if domain not in [None, 'edit', 'motion']:
            self.logger.error("Invalid domain specified for mastership release")
            return ("ERR - Invalid domain specified", -1)

        if domain:
            status = self.post_request(f"/rw/mastership/{domain}/release")
            return ("OK", status) if status == 204 else (f"ERR - Mastership on {domain} domain was not released", status)

        else:
            status = self.post_request("/rw/mastership/release")
            return ("OK", status) if status == 204 else ("ERR - Mastership on all domains was not released", status)

    def set_io_signal(self, signal_name: str, signal_value: str, network="", device="") -> tuple[str, int]:
        """Set the value of an I/O signal."""
        if not signal_name or signal_value is None:
            self.logger.error("Signal name (param1) or value (param2) cannot be empty")
            return ("ERR - Signal name (param1) or value (param2) cannot be empty", -1)
        if network:
            network = network if network.endswith('/') else network + '/'
        if device:
            device = device if device.endswith('/') else device + '/'


        path = f"/rw/iosystem/signals/{network}{device}{signal_name}/set-value"
        
        status = self.post_request(path, dataIn={'lvalue': signal_value})
        
        if status == 204:
            return ("OK", status)
        else:
            self.logger.error(f"Failed to set IO signal {signal_name}")
            return (f"ERR - Failed to set IO signal {signal_name}", status)
        

    def set_speedratio(self, speed_ratio: str)  -> tuple[str, int]:
        """Set the robot speed ratio (0-100)."""
        speed_ratio_int = int(speed_ratio)
        if not (0 <= speed_ratio_int <= 100):
            return ("ERR - Speed ratio must be between 0 and 100", -1)

        status = self.post_request("/rw/panel/speedratio", dataIn={'speed-ratio': speed_ratio})
        
        if status == 204:
            return ("OK", status)
        else:
            self.logger.error(f"Failed to set speed ratio to {speed_ratio}") 
            return (f"ERR - Failed to set speed ratio to {speed_ratio}", status)

    def reset_energy_info(self)  -> tuple[str, int]:
        """Reset accumulated energy information."""
        status = self.post_request("/rw/system/energy/reset")
        
        if status == 204:
            return ("OK", status)
        else:
            self.logger.error("Failed to reset energy information")
            return ("ERR - Failed to reset energy information", status)

    def set_rapid_symbol_raw(self, value: str, symbol_name: str, module_name: str, task_name: str = "T_ROB1")  -> tuple[str, int]:
        """Set a RAPID symbol value. Requires edit mastership."""
        if not symbol_name or not module_name:
            self.logger.error("Signal name (param1) or module name (param2) cannot be empty")
            return ("ERR - Signal name (param1) or module name (param2) cannot be empty", -1)

        symbol_url = f"RAPID%2F{task_name}%2F{module_name}%2F{symbol_name}"

        # value_quoted = f'"{value}"'  # RAPID strings are quoted

        status = self.post_request(f"/rw/rapid/symbol/{symbol_url}/data", dataIn={'value': value})

        if status == 204:
            return ("OK", status)
        elif status == 400:
            self.logger.error(f"ERR - String type has to be double quoted .e.g '\"{value}\"' ")
            return (f"ERR - String type has to be in format '\"{value}\"' ", status)

        else:
            self.logger.error(f"Failed to set RAPID symbol {symbol_name} in module {module_name}, ensure mastership on edit domain")
            return (f"ERR - Failed to set RAPID symbol {symbol_name} in module {module_name}, ensure mastership on edit domain", status)



    def create_dipc_queue(self, queue_name: str, queue_size: str, message_size: str) -> tuple[str, int]:
        """Create a DIPC queue with the given name and size."""
        if not queue_name:
            self.logger.error("Queue name cannot be empty")
            return ("ERR - Queue name cannot be empty", -1)
        
        if not queue_size.isdigit() or int(queue_size) <= 0:
            self.logger.error("Queue size must be a positive integer")
            return ("ERR - Queue size must be a positive integer", -1)

        if not message_size.isdigit() or int(message_size) <= 0:
            self.logger.error("Message size must be a positive integer")
            return ("ERR - Message size must be a positive integer", -1)

        datastr = f'dipc-queue-name={queue_name}&dipc-queue-size={queue_size}&dipc-max-msg-size={message_size}'
        # print(datastr)

        status = self.post_request("/rw/dipc", dataIn=datastr)

        if status == 201:
            return ("OK", status)
        else:
            self.logger.error(f"Failed to create DIPC queue {queue_name}")
            return (f"ERR - Failed to create DIPC queue {queue_name}", status)

    def send_dipc_message(self, message: str, userdef: str = "1",  queue_name: str = "RMQ_T_ROB1",) -> tuple[str, int]:
        """Send a message to a DIPC queue."""
        if not queue_name or not message:
            self.logger.error("Queue name and message cannot be empty")
            return ("ERR - Queue name and message cannot be empty", -1)
        
        if not userdef.isdigit() or not (0 <= int(userdef) <= 255):
            self.logger.error("Userdef must be an integer between 0 and 255")
            return ("ERR - Userdef must be an integer between 0 and 255", -1)
        
        payload={"dipc-src-queue-name": queue_name, "dipc-cmd": 0, "dipc-userdef": userdef,
                 "dipc-msgtype": 1, "dipc-data": message}
        

        # message_str = f"dipc-src-queue-name={queue_name}&dipc-cmd=111&dipc-userdef=222&dipc-msgtype=1&dipc-data={message}"
        
        status = self.post_request(f"/rw/dipc/{queue_name}/?action=dipc-send", dataIn=payload)

        
        if status == 204:
            return ("OK", status)
        else:
            # self.logger.info(f"Failed to send message to DIPC queue (QUEUE PROBABLY FULL){queue_name}")
            return (f"Failed to send message to DIPC queue (QUEUE PROBABLY FULL){queue_name}", status)



    # GET METHODS - non-string return types, for internal Python use
    def is_running(self):
        """ Returns True if the RAPID execution is running, False otherwise """
        (data, status) = self.get_rapid_execution_state()
        if status != 200:
            self.logger.error("Failed to get RAPID execution state")
            raise RWSException("Failed to get RAPID execution state")

        state = json.loads(data)[0]["ctrlexecstate"]

        if state == 'running':
            return True
        elif state == 'stopped':
            return False
        else:
            raise RWSException(f"Unknown RAPID execution state: {state}")
        
    def is_rapid_idle(self):
        """ Returns True if the RAPID execution is idle, False otherwise """

        if not self.is_running():
            return False

        (data, status) = self.get_rapid_symbol(RobotControllerConstants.Symbols.CURRENT_STATE, RobotControllerConstants.Modules.MAIN)
        if status != 200:
            self.logger.error("Failed to get RAPID symbol for current state")
            raise RWSException("Failed to get RAPID symbol for current state")
        else:

            if int(data) == 0: # RAPID symbol for idle state is 0
                return True
            else:
                return False
    
    def is_master(self, domain=None): 
        """ 
        Returns True if the client has mastership on given domain (Edit or Motion), False otherwise 
        """
        if domain not in ['edit', 'motion']:
            raise RWSException("Invalid domain specified")
        
        if domain:
            (data,status) = self.get_request(f"/rw/mastership/{domain}")
            mastership  = data.get('state', [])[0]
            held_by_me = mastership.get('mastershipheldbyme', False)
            if held_by_me:
                return True
            else:
                return False
              
    def make_robot_ready(self):
        """Prepare the robot for operation (mastership, motors, RAPID start)."""
        
        if not self.is_master('edit'):
            mess, code = self.request_mastership('edit')
            if code != 204:
                return ("ERR - Failed to obtain mastership on edit domain:", code)
            
        if self.is_master('motion'):
            mess, code = self.release_mastership('motion')
            if code != 204:
                return ("ERR - Failed to release mastership on motion domain:", code)
            
        if not self.is_running(): 
            mess, code = self.get_opmode_state()
            if mess != "auto":
                return (f"ERR - Failed controller is in {mess} mode, change to auto mode", code)
            
            mess, code = self.get_controller_state()

            if mess != "motoron":
                _, _ = self.motors_on()
                time.sleep(1) 

            mess, code = self.get_controller_state()
            if mess != "motoron":
                return (f"ERR - Failed to turn on motors: {mess}", code)
            
            mess, code = self.reset_pp()
            if code != 204:
                return (f"ERR - Failed to reset program pointer: {mess}", code)
            
            time.sleep(1)
            mess, code = self.get_rapid_execution_state()
            state = json.loads(mess)[0]["ctrlexecstate"]
            print(f"RAPID execution state: {state}")
            if state != "running":
                mess, code = self.start_rapid_script()
                if mess != "OK":
                    return (f"ERR - Failed to start RAPID script: {mess}", code)
                time.sleep(1) 

            mess, code = self.get_rapid_execution_state()
            state = json.loads(mess)[0]["ctrlexecstate"]
            if state == "running":
                return ("Robot set up correctly", code)
            else:
                return (f"ERR - Failed to start RAPID script: {mess}", code)
            
        else:
            return ("Robot is already running", 200)

    def run_move_command(self, motion_command: str, robtarget: str, speed: str) -> tuple[str, int]:
        """Execute a motion command (MoveL/MoveJ) to the given robtarget."""
        if self.is_rapid_idle():

            if motion_command not in [RobotControllerConstants.MotionCommands.MOVE_L, RobotControllerConstants.MotionCommands.MOVE_J]:
                return ("ERR - Unsupported motion command", -1)
            if motion_command == RobotControllerConstants.MotionCommands.MOVE_L:
                routine_name = RobotControllerConstants.Routines.SINGLE_MOVE_L
            elif motion_command == RobotControllerConstants.MotionCommands.MOVE_J:
                routine_name = RobotControllerConstants.Routines.SINGLE_MOVE_J

            self.set_rapid_symbol_raw(f'"{routine_name}"', RobotControllerConstants.Symbols.ROUTINE_NAME, RobotControllerConstants.Modules.RAPID)
            time.sleep(0.1)
            self.set_rapid_symbol_raw(speed, RobotControllerConstants.Symbols.SPEED, RobotControllerConstants.Modules.USER)
            self.set_rapid_symbol_raw(robtarget, RobotControllerConstants.Symbols.RECEIVED_ROBTARGET, RobotControllerConstants.Modules.USER)
            time.sleep(0.2) # small delay to ensure symbols are set before starting the routine
            self.set_rapid_symbol_raw(RobotControllerConstants.States.EXECUTE, RobotControllerConstants.Symbols.CURRENT_STATE, RobotControllerConstants.Modules.MAIN)
            return ("OK", 200)

        else:
            return ("ERR - Cannot execute motion command while RAPID is not idle", -1)
        

    def run_rapid_routine(self, routine_name: str) -> tuple[str, int]:
        """ Run a RAPID routine on the robot."""
        if self.is_rapid_idle():
            self.set_rapid_symbol_raw(f'"{routine_name}"', RobotControllerConstants.Symbols.ROUTINE_NAME, RobotControllerConstants.Modules.RAPID)
            time.sleep(0.2) # small delay to ensure symbols are set before starting the routine
            self.set_rapid_symbol_raw(RobotControllerConstants.States.EXECUTE, RobotControllerConstants.Symbols.CURRENT_STATE, RobotControllerConstants.Modules.MAIN)
            return ("OK", 200)

        else:
            return ("ERR - Cannot execute RAPID routine while RAPID is not idle", -1)


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

