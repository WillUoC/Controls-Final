class InitializationError(Exception):
    """Raised when there is an issue initializing a class"""

    def __init__(self, message="INITIALIZATION ERROR"):
        super().__init__(message)

class StateError(Exception):
    """Raised when there is an issue setting a state (in a Finite State Machine)"""

    def __init__(self, message="STATE ERROR"):
        super().__init__(message)