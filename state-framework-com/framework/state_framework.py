import struct
from enum import Enum
from typing import Any

from serial.serialutil import SerialException

from framework.data_type import DataType, get_data_type_size, get_data_type_struct_str
from framework.fault_definition import FaultDefinition
from framework.persistent_data_entry import PersistentDataEntry
from framework.readable.readable import Readable
from framework.registered_command import RegisteredCommand, CommandInputType
from framework.variable_definition import VariableDefinition
from serial_helper import read_string, read_fixed_string


class OutputPacket(Enum):
    LOG_MESSAGE = 1
    STATE_UPDATE = 2
    PERSISTENT_STATE_UPDATE = 3
    METADATA = 4,
    DEVICE_RESTART_MARKER = 5,
    FAULTS_CHANGED = 6
    PHASE_CHANGED = 7


class MetadataSegment(Enum):
    APPLICATION_NAME = 1
    COMMANDS = 2
    VARIABLE_DEFINITIONS = 3
    PERSISTENT_STORAGE_ENTRIES = 4
    FAULT_INFORMATION = 5
    INITIAL_PHASE = 6
    METADATA_END = 255


class LogLevel(Enum):
    DEBUG = 1
    INFO = 2
    WARNING = 3
    ERROR = 4
    SERIAL_ONLY = 5


class StateFramework:
    application_name: str

    commands: list[RegisteredCommand] = []
    persistent_entries: list[PersistentDataEntry] = []
    fault_definitions: list[FaultDefinition] = []

    current_phase_id: int = -1
    current_phase: str

    total_data_len = 0
    variable_definitions: list[VariableDefinition] = []
    state: dict[int, Any] = {}

    @staticmethod
    def generate_framework_configuration(readable: Readable):
        state_framework = StateFramework()
        received_segments: list[MetadataSegment] = []

        while True:
            if readable.bytes_avail() == 0:
                if readable.is_live_device():
                    continue
                else:
                    print('Could not read framework metadata')
                    break

            segment_id, = struct.unpack('<B', readable.read(1))

            try:
                metadata_segment = MetadataSegment(segment_id)
            except ValueError:
                continue

            if metadata_segment in received_segments:
                if readable.is_live_device():
                    print(f'Received duplicate segment {hex(segment_id)}, ignoring')
                    continue
                else:
                    print(f'Received duplicate segment {hex(segment_id)}, can not continue')
                    break

            received_segments.append(metadata_segment)

            match metadata_segment:
                case MetadataSegment.APPLICATION_NAME:
                    state_framework._handle_segment_application_name(readable)
                case MetadataSegment.COMMANDS:
                    state_framework._handle_segment_commands(readable)
                case MetadataSegment.VARIABLE_DEFINITIONS:
                    state_framework._handle_segment_variable_definitions(readable)
                case MetadataSegment.PERSISTENT_STORAGE_ENTRIES:
                    state_framework._handle_segment_persistent_storage(readable)
                case MetadataSegment.FAULT_INFORMATION:
                    state_framework._handle_segment_fault_information(readable)
                case MetadataSegment.INITIAL_PHASE:
                    state_framework._update_phase(readable)
                case MetadataSegment.METADATA_END:
                    return state_framework
                case _:
                    print(f'Unimplemented metadata segment: {segment_id} ({hex(segment_id)})')
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
            case LogLevel.SERIAL_ONLY:
                prefix = 'SERIAL: '
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
                    case OutputPacket.PERSISTENT_STATE_UPDATE:
                        self._update_persistent_data(readable)
                    case OutputPacket.DEVICE_RESTART_MARKER:
                        print('Device restarted!!')
                    case OutputPacket.FAULTS_CHANGED:
                        self._update_faults(readable)
                    case OutputPacket.PHASE_CHANGED:
                        self._update_phase(readable)
                    case _:
                        print(f'Unknown output packet: {output_packet} ({hex(packet_id)})')
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

    def _handle_segment_persistent_storage(self, readable: Readable):
        str_entries: list[PersistentDataEntry] = []
        non_str_entries: list[PersistentDataEntry] = []

        entry_count, offset_size = struct.unpack('<2B', readable.read(2))
        for _ in range(entry_count):
            new_entry = PersistentDataEntry.read_entry(readable, offset_size)
            if new_entry.data_type == DataType.STRING:
                str_entries.append(new_entry)
            else:
                non_str_entries.append(new_entry)

        str_entries.sort(key=lambda pde: pde.offset)
        non_str_entries.sort(key=lambda pde: pde.offset)
        self.persistent_data_entries = non_str_entries + str_entries

        self._update_persistent_data(readable)

    def _handle_segment_fault_information(self, readable: Readable):
        def_count, all_faults = struct.unpack('<BI', readable.read(5))
        for _ in range(def_count):
            self.fault_definitions.append(FaultDefinition.read_definition(readable, all_faults))

    def _update_persistent_data(self, readable: Readable):
        tag, = struct.unpack('<I', readable.read(4))
        for entry in self.persistent_data_entries:
            if entry.data_type == DataType.STRING:
                entry.current_value = read_string(readable)
            else:
                data = readable.read(get_data_type_size(entry.data_type))
                entry.current_value, = struct.unpack(get_data_type_struct_str(entry.data_type), data)

        for entry in self.persistent_data_entries:
            print(f'{entry.display_name} = {entry.current_value} ({entry.offset})')


    def _update_faults(self, readable: Readable):
        changed_fault_bit, all_faults = struct.unpack('<BI', readable.read(5))
        change_message = read_string(readable)

        for fault in self.fault_definitions:
            fault.is_faulted = ((all_faults >> fault.fault_bit) & 0x01) > 0
            if fault.fault_bit == changed_fault_bit:
                print(
                    f'Fault {fault.fault_name} (bit: {changed_fault_bit}) is changed (now {fault.is_faulted}): {change_message}')

    def _update_phase(self, readable: Readable):
        old_phase_id = self.current_phase_id
        self.current_phase_id, = struct.unpack('<B', readable.read(1))
        self.current_phase = read_string(readable)
        print(f'Phase{'' if old_phase_id < 0 else ' changed'}: {self.current_phase} ({self.current_phase_id})')