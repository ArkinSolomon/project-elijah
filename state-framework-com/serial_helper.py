import struct

from framework.readable.readable import Readable

def read_string(tty: Readable) -> str:
    string = ''
    while True:
        data = tty.read(1)
        char, = struct.unpack('<B', data)
        if chr(char) == '\0':
            break
        else:
            string += chr(char)
    return string


def read_fixed_string(tty: Readable, string_length: int) -> str:
    return tty.read(string_length).decode("utf-8", errors='replace')
