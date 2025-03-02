import struct

import serial

from framework.data_type import DataType, get_data_type_size, get_data_type_struct_str
from framework.registered_command import RegisteredCommand, CommandInputType
from framework.variable_definition import VariableDefinition
from serial_helper import read_string


class MetadataSegments:
    ApplicationName = 1
    Commands = 2
    VariableDefinitions = 3
    MetadataEnd = 255


class StateFramework:
    application_name: str

    commands: list[RegisteredCommand] = []

    total_data_len = 0
    variable_definitions: list[VariableDefinition] = []
    state = {}

    @staticmethod
    def generate_framework_configuration(tty: serial.Serial):
        metadata = StateFramework()
        while True:
            if tty.in_waiting < 1:
                continue
            segment_id, = struct.unpack('<B', tty.read(1))

            match segment_id:
                case MetadataSegments.ApplicationName:
                    metadata._handle_segment_application_name(tty)
                case MetadataSegments.Commands:
                    metadata._handle_segment_commands(tty)
                case MetadataSegments.VariableDefinitions:
                    metadata._handle_segment_variable_definitions(tty)
                case MetadataSegments.MetadataEnd:
                    return metadata
                case _:
                    print(f'Unknown segment id: {segment_id} ({hex(segment_id)})')

    def state_updated(self, tty: serial.Serial):
        data = tty.read(self.total_data_len)
        for var_def in self.variable_definitions:
            var_size = get_data_type_size(var_def.data_type)
            value = struct.unpack(get_data_type_struct_str(var_def.data_type),
                                  data[var_def.data_offset:var_def.data_offset + var_size])[0]
            self.state[var_def.variable_id] = value

    def _handle_segment_application_name(self, tty: serial.Serial):
        self.application_name = read_string(tty)

    def _handle_segment_commands(self, tty: serial.Serial):
        num_commands, = struct.unpack('<B', tty.read(1))
        for _ in range(num_commands):
            command_id, command_input = struct.unpack('<2B', tty.read(2))
            command_name = read_string(tty)
            registered_command = RegisteredCommand(command_id, command_name, CommandInputType(command_input))
            self.commands.append(registered_command)
            print(registered_command)

    def _handle_segment_variable_definitions(self, tty):
        num_vars, offset_size = struct.unpack('<2B', tty.read(2))

        offset_unit = {
            1: 'B',
            2: 'H',
            4: 'I',
            8: 'Q'
        }.get(offset_size, 'I')

        for _ in range(num_vars):
            var_id, var_offset, data_type_id = struct.unpack(f'<B{offset_unit}B', tty.read(1 + offset_size + 1))
            disp_name = read_string(tty)
            disp_unit = read_string(tty)
            data_type = DataType(data_type_id)
            var_def = VariableDefinition(var_id, disp_name, disp_unit, var_offset, data_type)

            self.total_data_len += get_data_type_size(data_type)
            self.variable_definitions.append(var_def)

            # TODO defaults for differnt types
            self.state[var_id] = 0
