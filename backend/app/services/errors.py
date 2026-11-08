"""Domain errors raised by services; the API layer maps `code` to HTTP status."""


class ServiceError(Exception):
    def __init__(self, code: str, message: str) -> None:
        super().__init__(message)
        self.code = code
        self.message = message


class NotFound(ServiceError):
    def __init__(self, what: str) -> None:
        super().__init__("NOT_FOUND", f"{what} not found")


class InvalidState(ServiceError):
    def __init__(self, message: str) -> None:
        super().__init__("INVALID_STATE", message)


class PlanningFailed(ServiceError):
    def __init__(self, message: str) -> None:
        super().__init__("PLANNING_FAILED", message)
