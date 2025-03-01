import struct

import serial

from framework.data_type import DataType
from framework.registered_command import RegisteredCommand, CommandInputType
from framework.variable_definition import VariableDefinition
from serial_helper import read_string


class MetadataSegments:
    ApplicationName = 1
    Commands = 2
    VariableDefinitions = 3
    MetadataEnd = 255


class FrameworkMetadata:
    application_name: str

    commands: [RegisteredCommand] = []
    variable_definitions: [VariableDefinition] = []

    @staticmethod
    def generate_framework_configuration(tty: serial.Serial):
        metadata = FrameworkMetadata()
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

    def _handle_segment_application_name(self, tty):
        self.application_name = read_string(tty)

    def _handle_segment_commands(self, tty):
        num_commands, = struct.unpack('<B', tty.read(1))
        for _ in range(num_commands):
            command_id, command_input = struct.unpack('<2B', tty.read(2))
            command_name = read_string(tty)
            registered_command = RegisteredCommand(command_id, command_name, CommandInputType(command_input))
            self.commands.append(registered_command)

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
            var_def = VariableDefinition(var_id, disp_name, disp_unit, var_offset, DataType(data_type_id))
            self.variable_definitions.append(var_def)