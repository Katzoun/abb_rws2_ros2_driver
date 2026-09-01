"""The RAPID vocabulary this node speaks.
"""


class RobotControllerConstants:

    class MotionCommands:
        MOVE_L = "MoveL"
        MOVE_J = "MoveJ"
        MOVE_ABS_J = "MoveAbsJ"
        MOVE_ABS_L = "MoveAbsL"

    class Modules:
        RAPID = "TRobRAPID"
        USER = "TRobUser"
        MAIN = "TRobMain"

    class Symbols:
        ROUTINE_NAME = "routine_name_input"
        SPEED = "speednum"
        CURRENT_STATE = "current_state"
        RECEIVED_ROBTARGET = "received_robtarget"

    class States:
        EXECUTE = "2"

    class Routines:
        MOVE_L = "run_routine_buffer_moveL"
        MOVE_J = "run_routine_buffer_moveJ"
        MOVE_ABS_J = "run_routine_buffer_moveabsJ"
        MOVE_ABS_L = "run_routine_buffer_moveabsL"
        SINGLE_MOVE_L = "run_single_moveL"
        SINGLE_MOVE_J = "run_single_moveJ"
