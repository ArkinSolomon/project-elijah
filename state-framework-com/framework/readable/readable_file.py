from io import BufferedReader

from framework.readable.readable import Readable


class ReadableFile(Readable):

    file: BufferedReader
    file_size: int

    def __init__(self, file: BufferedReader):
        self.file = file

        curr_pos = self.file.tell()
        self.file.seek(0, 2)
        self.file_size = self.file.tell()
        self.file.seek(curr_pos)


    def is_live_device(self) -> bool:
        return False

    def bytes_avail(self) -> int:
       return self.file_size - self.file.tell()

    def read(self, n: int) -> bytes:
        return self.file.read(n)
