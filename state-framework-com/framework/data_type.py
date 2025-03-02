from enum import Enum


class DataType(Enum):
    STRING = 1
    INT8 = 2
    UINT8 = 3
    INT16 = 4
    UINT16 = 5
    INT32 = 6
    UINT32 = 7
    INT64 = 8
    UINT64 = 9
    FLOAT = 10
    DOUBLE = 11
    TIME = 12


def get_data_type_size(data_type: DataType) -> int:
    match data_type:
        case DataType.STRING:
            return 0
        case DataType.INT8 | DataType.UINT8:
            return 1
        case DataType.INT16 | DataType.UINT16:
            return 2
        case DataType.INT32 | DataType.UINT32 | DataType.FLOAT:
            return 4
        case DataType.INT64 | DataType.UINT64 | DataType.DOUBLE:
            return 8
        case DataType.TIME:
            return 36


def get_data_type_struct_str(data_type: DataType) -> str:
    match data_type:
        case DataType.STRING:
            raise Exception('Can not get a struct string of a string')
        case DataType.INT8:
            return '<b'
        case DataType.UINT8:
            return '<B'
        case DataType.INT16:
            return '<h'
        case DataType.UINT16:
            return '<H'
        case DataType.INT32:
            return '<i'
        case DataType.UINT32:
            return '<I'
        case DataType.FLOAT:
            return '<f'
        case DataType.INT64:
            return '<q'
        case DataType.UINT64:
            return '<Q'
        case DataType.DOUBLE:
            return '<d'
        case DataType.TIME:
            # TODO encoding times
            raise Exception('Can not get a struct string of a time (yet)')
