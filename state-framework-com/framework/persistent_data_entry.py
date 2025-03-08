import struct
from typing import Any

from framework.data_type import DataType
from framework.readable.readable import Readable
from serial_helper import read_string


class PersistentDataEntry:
    key_id: int
    data_type: DataType
    offset: int
    display_name: str

    current_value: Any

    @staticmethod
    def read_entry(readable: Readable, offset_bytes: int) -> 'PersistentDataEntry':
        offset_unit = {
            1: 'B',
            2: 'H',
            4: 'I',
            8: 'Q'
        }.get(offset_bytes, 'I')

        entry = PersistentDataEntry()
        entry.key_id, data_type_id, entry.offset = struct.unpack(f'<BB{offset_unit}', readable.read(2 + offset_bytes))
        entry.data_type = DataType(data_type_id)
        entry.display_name = read_string(readable)

        return entry