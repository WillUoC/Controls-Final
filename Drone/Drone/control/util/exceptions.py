class InitializationError(Exception):
    """Raised when there is an issue initializing a class"""

    def __init__(self, message="INITIALIZATION ERROR"):
        self.message = "INITIALIZATION ERROR"
        super().__init__(self.message)