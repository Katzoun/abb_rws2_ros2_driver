"""A stand-in for the robot controller.

Same surface as RWSInterface, but nothing leaves the process. It exists so the
whole pipeline - trigger, analysis, path, motion - can be run and watched on a
desk with no GoFa and no controller.

It keeps every robtarget it is sent, so what the robot would have drawn can be
dumped and looked at afterwards.
"""

import json
import re
import time

from robot_control.constants import RapidConfig, RoutineNames


class SimulatedRWS:
    """Answers like RWSInterface does: (message, http_status) tuples."""

    # How long a triggered RAPID routine pretends to run. The real program flips
    # current_state back to 0 when it finishes and callers poll for that, so a
    # simulation that returned idle immediately would hide every race.
    ROUTINE_DURATION_S = 1.5

    def __init__(self, host=None, username=None, password=None, port=None, logger=None,
                 timeout_s=None, rapid=None, routines=None):
        self.logger = logger
        # Same configuration surface as RWSInterface, so the node can build
        # either one without caring which it got.
        self.rapid = rapid if rapid is not None else RapidConfig()
        self.routines = routines if routines is not None else RoutineNames()
        self._logged_in = False
        self._busy_until = 0.0
        self._symbols = {}
        self._routine = None
        # Every robtarget/jointtarget handed over, in order.
        self.received = []

    def _log(self, message):
        if self.logger:
            self.logger.info(f"[sim] {message}")

    # ---- session ----

    def login(self):
        self._logged_in = True
        self._log("logged in to the simulated controller")
        return True

    def logout(self):
        self._logged_in = False
        return True

    def get_login_state(self):
        return self._logged_in

    def send_keepalive(self):
        return True

    # ---- state ----

    def is_running(self):
        return True

    def is_rapid_idle(self):
        return time.monotonic() >= self._busy_until

    def get_rapid_symbol(self, symbol_name, module_name):
        if symbol_name == self.rapid.symbol_current_state:
            return (self.rapid.state_idle if self.is_rapid_idle() else self.rapid.state_execute, 200)
        return (self._symbols.get((symbol_name, module_name), ""), 200)

    def get_robot_joint_positions(self):
        # A plausible resting pose; enough for anything watching joint states.
        resting = [0.0, -20.0, 30.0, 0.0, 70.0, 0.0]
        joints = {f"rax_{i+1}": (resting[i] if i < len(resting) else 0.0)
                  for i in range(self.rapid.num_axes)}
        return (json.dumps(joints), 200)

    def get_controller_state(self):
        return ("motoron", 200)

    def get_opmode_state(self):
        return ("auto", 200)

    def get_rapid_execution_state(self):
        return (json.dumps([{"ctrlexecstate": "running", "cycle": "once"}]), 200)

    # ---- commands ----

    def set_rapid_symbol_raw(self, value, symbol_name, module_name) -> None:
        self._symbols[(symbol_name, module_name)] = value
        if symbol_name == self.rapid.symbol_current_state and str(value).strip('"') == self.rapid.state_execute:
            self._busy_until = time.monotonic() + self.ROUTINE_DURATION_S
            self._log(f"running routine {self._routine!r}")
        if symbol_name == self.rapid.symbol_routine_name:
            self._routine = str(value).strip('"')

    def send_dipc_message(self, message, userdef="1"):
        self.received.append({"message": message, "userdef": userdef})
        if userdef == "2":
            self._log(f"last point of the path ({len(self.received)} in total)")
        return True

    def make_robot_ready(self):
        self._log("robot ready")
        return "Robot set up correctly"

    def run_rapid_routine(self, routine_name) -> None:
        self._routine = routine_name
        self._busy_until = time.monotonic() + self.ROUTINE_DURATION_S
        self._log(f"routine {routine_name}")

    def run_move_command(self, motion_command, robtarget, speed):
        self.received.append({"message": robtarget, "motion": motion_command, "speed": speed})
        self._busy_until = time.monotonic() + self.ROUTINE_DURATION_S
        self._log(f"{motion_command} at speed {speed} to {robtarget[:60]}")

    def set_speedratio(self, speed_ratio) -> None:
        # Same guard as the real one, or testing against the stand-in would miss
        # a bad value that the controller would refuse.
        if not 0 <= int(speed_ratio) <= 100:
            raise ValueError(f"Speed ratio {speed_ratio} is outside 0-100")
        self._log(f"speed ratio {speed_ratio}")

    def request_mastership(self, domain=None) -> None:
        pass

    def release_mastership(self, domain=None) -> None:
        pass

    def motors_on(self) -> None:
        pass

    def motors_off(self) -> None:
        pass

    # ---- what would have been drawn ----

    POINT_RE = re.compile(r"\[\[([-\d.eE+]+),([-\d.eE+]+),([-\d.eE+]+)\]")

    def points(self):
        """The xyz of every robtarget received, in the order they arrived."""
        out = []
        for entry in self.received:
            m = self.POINT_RE.search(entry["message"])
            if m:
                out.append(tuple(float(g) for g in m.groups()))
        return out

    def dump_points(self, path):
        """Same (message, status) shape as every other call here, so it can be
        reached through controller_request like the rest."""
        pts = self.points()
        with open(path, "w") as f:
            f.write("x,y,z\n")
            for x, y, z in pts:
                f.write(f"{x:.3f},{y:.3f},{z:.3f}\n")
        self._log(f"wrote {len(pts)} points to {path}")
        return (f"wrote {len(pts)} points to {path}", 200)
