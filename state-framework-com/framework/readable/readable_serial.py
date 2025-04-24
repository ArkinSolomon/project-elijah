import serial

from framework.readable.readable import Readable


class ReadableSerial(Readable):
    tty: serial.Serial
    out_file: str | None = None

    def __init__(self, tty: serial.Serial, out_file: str | None = None):
        self.tty = tty
        self.out_file = out_file

    def is_live_device(self) -> bool:
        return True

    def bytes_avail(self) -> int:
        return self.tty.in_waiting

    def read(self, n: int) -> bytes:
        read_data = self.tty.read(n)
        if self.out_file is not None:
            with open(self.out_file, 'ab+') as out_file:
                out_file.write(read_data)
        return read_data

    def write(self, data: bytes) -> int | None:
        return self.tty.write(data)