"""What can go wrong talking to the controller.

Three classes, not because a hierarchy is tidy but because a caller does three
different things: retry, give up, or tell the operator. Anything finer than that
had no caller and would just be ceremony.
"""


class RWSError(Exception):
    """Robot Web Services call failed. Carries the HTTP status where there was one."""

    def __init__(self, message: str, status_code: int = -1):
        super().__init__(message)
        self.status_code = status_code


class RWSConnectionError(RWSError):
    """The request never got through - timeout, refused, no route.

    Nothing happened on the controller, so retrying or logging in again is
    reasonable.
    """


class RWSStateError(RWSError):
    """The controller answered, and said no.

    Wrong operating mode, motors off, mastership held elsewhere, program not
    running. Retrying the same call changes nothing; something has to change on
    the robot first.
    """
