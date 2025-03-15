import struct
import time

def encode_time(time_inst) -> bytes:
    full_year = time_inst.tm_year + 1900
    return bytes([
        time_inst.tm_sec,
        time_inst.tm_min,
        time_inst.tm_hour,
        time_inst.tm_wday,
        time_inst.tm_mday,
        time_inst.tm_mon
    ]) + struct.pack('<H', full_year)


def decode_time(encoded_time_inst: bytes) -> time.struct_time:
    seconds, minutes, hours, wday, mday, mon = encoded_time_inst[:6]
    full_year = struct.unpack('<H', encoded_time_inst[6:8])[0]

    return time.struct_time((full_year, mon, mday, hours, minutes, seconds, wday, 0, -1))
