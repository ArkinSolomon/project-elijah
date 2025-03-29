import time
from enum import Enum
from datetime import datetime

class LogLevel(Enum):
    DEBUG = 1
    INFO = 2
    WARNING = 3
    ERROR = 4
    SERIAL_ONLY = 5
    SYSTEM = 257

class LogMessage:
    log_level: LogLevel
    message: str
    recv_time: datetime

    def __init__(self, log_level: LogLevel, message: str):
        self.log_level = log_level
        self.message = message
        self.recv_time = datetime.now()

    def __str__(self) -> str:
        return f"{self.get_log_header_str()}{self.message}"

    def get_log_header_str(self) -> str:
        time_str = self.recv_time.strftime("%H:%M:%S")
        log_type: str = "System"
        match self.log_level:
            case LogLevel.DEBUG:
                log_type = "Debug"
            case LogLevel.INFO:
                log_type = "Info"
            case LogLevel.WARNING:
                log_type = "Warning"
            case LogLevel.ERROR:
                log_type = "Error"
            case LogLevel.SERIAL_ONLY:
                log_type = "Serial"
        return f"[{time_str}][{log_type:7}]: "