"""The vocabulary the driver shares with the RAPID program on the controller.

Module names, symbol names, routine names and the state values written into
them are a matched pair with the code running on the robot - change one and the
other has to change with it. That is why they live here as constants rather
than as ROS parameters: they are a contract, not a setting.

Taken unchanged from the intra project's core_pkg/systemconstants.py.
"""


class RobotControllerConstants:
    NODE_NAME = "robot_controller"

    class ServiceNames:
        CONTROLLER_REQUEST = "controller_request"

    class TopicNames:
        JOINT_STATES_TOPIC = "joint_states"

    class ActionNames:
        ROBOT_ROBTARGET_MOVE_ACTION = "robot_robtarget_move"
        ROBOT_JOINTTARGET_MOVE_ACTION = "robot_jointtarget_move"

    class MotionCommands:
        MOVE_L     = "MoveL"
        MOVE_J     = "MoveJ"
        MOVE_ABS_J = "MoveAbsJ"
        MOVE_ABS_L = "MoveAbsL"

    class Modules:
        RAPID = "TRobRAPID"
        USER  = "TRobUser"
        MAIN  = "TRobMain"

    class Symbols:
        ROUTINE_NAME       = "routine_name_input"
        SPEED              = "speednum"
        CURRENT_STATE      = "current_state"
        RECEIVED_ROBTARGET = "received_robtarget"

    class States:
        EXECUTE = "2"
        IDLE   = "0"

    class Routines:
        MOVE_L        = "run_routine_buffer_moveL"
        MOVE_J        = "run_routine_buffer_moveJ"
        MOVE_ABS_J    = "run_routine_buffer_moveabsJ"
        MOVE_ABS_L    = "run_routine_buffer_moveabsL"
        SINGLE_MOVE_L = "run_single_moveL"
        SINGLE_MOVE_J = "run_single_moveJ"
        SINGLE_MOVE_C = "run_single_moveC"
