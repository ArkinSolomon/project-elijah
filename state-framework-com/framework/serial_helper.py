import struct

from framework.readable.readable import Readable

def read_string(readable: Readable) -> str:
    string = ''
    while True:
        data = readable.read(1)
        char, = struct.unpack('<B', data)
        if chr(char) == '\0':
            break
        else:
            string += chr(char)
    return string


def read_fixed_string(readable: Readable, string_length: int) -> str:
    return readable.read(string_length).decode("utf-8", errors='replace')
