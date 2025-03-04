import struct
from enum import Enum
from typing import Any

from serial.serialutil import SerialException

from framework.data_type import DataType, get_data_type_size, get_data_type_struct_str
from framework.readable.readable import Readable
from framework.registered_command import RegisteredCommand, CommandInputType
from framework.variable_definition import VariableDefinition
from serial_helper import read_string, read_fixed_string


class OutputPacket(Enum):
    LOG_MESSAGE = 1
    STATE_UPDATE = 2
    PERSISTENT_STATE_UPDATE = 3


class MetadataSegments(Enum):
    APPLICATION_NAME = 1
    COMMANDS = 2
    VARIABLE_DEFINITIONS = 3
    METADATA_END = 255


class LogLevel(Enum):
    DEBUG = 0
    INFO = 1
    WARNING = 2
    ERROR = 3


class StateFramework:
    application_name: str

    commands: list[RegisteredCommand] = []

    total_data_len = 0
    variable_definitions: list[VariableDefinition] = []
    state: dict[int, Any] = {}

    @staticmethod
    def generate_framework_configuration(readable: Readable):
        metadata = StateFramework()
        while True:
            if readable.bytes_avail() == 0:
                if readable.is_live_device():
                    continue
                else:
                    print('Could not read framework metadata')
                    break
            segment_id, = struct.unpack('<B', readable.read(1))

            match segment_id:
                case MetadataSegments.APPLICATION_NAME.value:
                    metadata._handle_segment_application_name(readable)
                case MetadataSegments.COMMANDS.value:
                    metadata._handle_segment_commands(readable)
                case MetadataSegments.VARIABLE_DEFINITIONS.value:
                    metadata._handle_segment_variable_definitions(readable)
                case MetadataSegments.METADATA_END.value:
                    return metadata
                case _:
                    print(f'Unknown segment id: {segment_id} ({hex(segment_id)})')
                    if not readable.is_live_device():
                        break

    @staticmethod
    def log(log_level: LogLevel, message: str):
        match log_level:
            case LogLevel.INFO:
                prefix = 'INFO: '
            case LogLevel.DEBUG:
                prefix = 'DEBUG: '
            case LogLevel.WARNING:
                prefix = 'WARNING: '
            case LogLevel.ERROR:
                prefix = 'ERROR: '
            case _:
                prefix = 'UNKNOWN: '
        print(f'{prefix}{message}')

    def update(self, readable: Readable, max_updates: int) -> int:
        packets_read = 0
        while packets_read < max_updates:
            if readable.bytes_avail() == 0:
                break

            try:
                packet_id, = struct.unpack('<B', readable.read(1))
                output_packet = OutputPacket(packet_id)
                packets_read += 1

                match output_packet:
                    case OutputPacket.LOG_MESSAGE:
                        log_level_id, message_length = struct.unpack('<BH', readable.read(3))
                        log_level = LogLevel(log_level_id)
                        log_message = read_fixed_string(readable, message_length)
                        self.log(log_level, log_message)
                    case OutputPacket.STATE_UPDATE:
                        self.state_updated(readable)
                        print(self.state)
                    case _:
                        print(f'Unknown output packet: {output_packet} ({packet_id})')
            except SerialException as e:
                raise e
            except Exception as e:
                continue

        return packets_read

    def state_updated(self, readable: Readable):
        data = readable.read(self.total_data_len)
        for var_def in self.variable_definitions:
            var_size = get_data_type_size(var_def.data_type)
            value = struct.unpack(get_data_type_struct_str(var_def.data_type),
                                  data[var_def.data_offset:var_def.data_offset + var_size])[0]
            self.state[var_def.variable_id] = value

    def _handle_segment_application_name(self, readable: Readable):
        self.application_name = read_string(readable)

    def _handle_segment_commands(self, readable: Readable):
        num_commands, = struct.unpack('<B', readable.read(1))
        for _ in range(num_commands):
            command_id, command_input = struct.unpack('<2B', readable.read(2))
            command_name = read_string(readable)
            registered_command = RegisteredCommand(command_id, command_name, CommandInputType(command_input))
            self.commands.append(registered_command)
            print(registered_command)

    def _handle_segment_variable_definitions(self, readable: Readable):
        num_vars, offset_size = struct.unpack('<2B', readable.read(2))

        offset_unit = {
            1: 'B',
            2: 'H',
            4: 'I',
            8: 'Q'
        }.get(offset_size, 'I')

        for _ in range(num_vars):
            var_id, var_offset, data_type_id = struct.unpack(f'<B{offset_unit}B', readable.read(1 + offset_size + 1))
            disp_name = read_string(readable)
            disp_unit = read_string(readable)
            data_type = DataType(data_type_id)
            var_def = VariableDefinition(var_id, disp_name, disp_unit, var_offset, data_type)

            self.total_data_len += get_data_type_size(data_type)
            self.variable_definitions.append(var_def)

            # TODO defaults for different types
            self.state[var_id] = 0
