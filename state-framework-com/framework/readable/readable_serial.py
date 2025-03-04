import serial

from framework.readable.readable import Readable


class ReadableSerial(Readable):
    tty: serial.Serial

    def __init__(self, tty: serial.Serial):
        self.tty = tty

    def is_live_device(self) -> bool:
        return True

    def bytes_avail(self) -> int:
        return self.tty.in_waiting

    def read(self, n: int) -> bytes:
        return self.tty.read(n)

    def write(self, data: bytes) -> int | None:
        return self.tty.write(data)