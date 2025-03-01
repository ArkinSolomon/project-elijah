import struct

import serial


def read_string(tty: serial.Serial) -> str:
    string = ''
    while True:
        data = tty.read(1)
        char, = struct.unpack('<B', data)
        if chr(char) == '\0':
            break
        else:
            string += chr(char)
    return string