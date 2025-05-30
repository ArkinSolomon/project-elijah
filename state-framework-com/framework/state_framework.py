import struct
from enum import Enum
from typing import Any, Tuple

from serial.serialutil import SerialException

import framework.time_helper as time_helper
from framework.data_type import DataType, get_data_type_size, get_data_type_struct_str
from framework.fault_definition import FaultDefinition
from framework.log_message import LogLevel, LogMessage
from framework.persistent_data_entry import PersistentDataEntry
from framework.readable.readable import Readable
from framework.registered_command import RegisteredCommand, CommandInputType
from framework.serial_helper import read_string, read_fixed_string
from framework.variable_definition import VariableDefinition


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


class StateFramework:
    application_name: str

    commands: list[RegisteredCommand]
    persistent_entries: list[PersistentDataEntry]
    fault_definitions: list[FaultDefinition]
    last_updated_fault: FaultDefinition | None

    current_phase_id: int = -1
    current_phase: str

    total_data_len: int
    variable_definitions: list[VariableDefinition]
    state: dict[int, Any]

    def __init__(self):
        self.application_name = None
        self.commands = []
        self.persistent_entries = []
        self.fault_definitions = []
        self.last_updated_fault = None
        self.current_phase_id = -1
        self.current_phase = "Unknown"
        self.total_data_len = 0
        self.variable_definitions = []
        self.state = {}

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
                    continue
                else:
                    raise Exception(f'Received duplicate segment {hex(segment_id)}, can not continue')

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

    def update(self, readable: Readable, max_updates: int) -> Tuple[int, bool, list[LogMessage]]:
        packets_read = 0
        state_changed = False
        logs: list[LogMessage] = []
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
                        message = read_fixed_string(readable, message_length)
                        log_message = LogMessage(log_level, message)
                        logs.append(log_message)
                    case OutputPacket.STATE_UPDATE:
                        state_changed = True
                        self.state_updated(readable)
                    case OutputPacket.PERSISTENT_STATE_UPDATE:
                        logs += self._update_persistent_data(readable)
                    case OutputPacket.DEVICE_RESTART_MARKER:
                        logs.append(LogMessage(LogLevel.SYSTEM, "Encountered device restart marker!"))
                    case OutputPacket.FAULTS_CHANGED:
                        fault_msg = self._update_faults(readable)
                        if fault_msg is not None:
                            logs.append(fault_msg)
                    case OutputPacket.PHASE_CHANGED:
                        logs.append(self._update_phase(readable))
                    case _:
                        print(f'Unknown output packet: {output_packet} ({hex(packet_id)})')
            except SerialException as e:
                raise e
            except Exception as e:
                continue

        return packets_read, state_changed, logs

    def state_updated(self, readable: Readable):
        data = readable.read(self.total_data_len)
        for var_def in self.variable_definitions:
            if var_def.data_type != DataType.TIME:
                var_size = get_data_type_size(var_def.data_type)
                value = struct.unpack(get_data_type_struct_str(var_def.data_type),
                                      data[var_def.data_offset:var_def.data_offset + var_size])[0]
                self.state[var_def.variable_id] = value
            else:
                self.state[var_def.variable_id] = time_helper.decode_time(
                    data[var_def.data_offset:var_def.data_offset + get_data_type_size(DataType.TIME)])

    def _handle_segment_application_name(self, readable: Readable):
        self.application_name = read_string(readable)

    def _handle_segment_commands(self, readable: Readable):
        num_commands, = struct.unpack('<B', readable.read(1))
        for _ in range(num_commands):
            command_id, command_input = struct.unpack('<2B', readable.read(2))
            command_name = read_string(readable)
            input_prompt = read_string(readable)
            registered_command = RegisteredCommand(command_id, command_name, input_prompt,
                                                   CommandInputType(command_input))
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

            if data_type == DataType.TIME:
                self.state[var_id] = None
            else:
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
        self.persistent_entries = non_str_entries + str_entries

        self._update_persistent_data(readable)

    def _handle_segment_fault_information(self, readable: Readable):
        def_count, all_faults = struct.unpack('<BI', readable.read(5))
        for _ in range(def_count):
            self.fault_definitions.append(FaultDefinition.read_definition(readable, all_faults))
        self._update_last_fault()

    def _update_persistent_data(self, readable: Readable) -> list[LogMessage]:
        tag, = struct.unpack('<I', readable.read(4))
        total_size = 0
        log_messages = []
        for entry in self.persistent_entries:
            if entry.data_type == DataType.STRING:
                entry.current_value = read_string(readable)
                total_size += len(entry.current_value) + 1
            elif entry.data_type == DataType.TIME:
                entry.current_value = time_helper.decode_time(readable.read(get_data_type_size(DataType.TIME)))
                total_size += get_data_type_size(DataType.TIME)
            else:
                data = readable.read(get_data_type_size(entry.data_type))
                entry.current_value, = struct.unpack(get_data_type_struct_str(entry.data_type), data)
                total_size += get_data_type_size(entry.data_type)

        log_messages.append(LogMessage(LogLevel.SYSTEM, f"Persistent state changed! (read {total_size} bytes + 4 tag bytes)"))
        # for entry in self.persistent_entries:
        #     log_messages.append(LogMessage(LogLevel.SYSTEM, f'{entry.display_name} = {entry.current_value} (offset: {entry.offset})'))
        return log_messages

    def _update_faults(self, readable: Readable) -> LogMessage | None:
        changed_fault_bit, all_faults = struct.unpack('<BI', readable.read(5))
        change_message = read_string(readable)
        change_log: LogMessage | None = None

        for fault in self.fault_definitions:
            fault.is_faulted = ((all_faults >> fault.fault_bit) & 0x01) > 0
            if fault.fault_bit == changed_fault_bit:
                if self.last_updated_fault is None or self.last_updated_fault.fault_bit == changed_fault_bit or not self.last_updated_fault.is_faulted:
                    self.last_updated_fault = fault
                fault.last_fault_message = change_message
                change_log = LogMessage(LogLevel.SYSTEM,
                                        f'Fault changed: {fault.fault_name} {'FAULT' if fault.is_faulted else 'OK'} (bit {fault.fault_bit}) [{hex(all_faults)}]: {'<no message>' if change_message == '' else change_message}')
            # if fault.fault_bit == changed_fault_bit:
            #     print(
            #         f'Fault {fault.fault_name} (bit: {changed_fault_bit}) is changed (now {fault.is_faulted}): {change_message}')
        self._update_last_fault()
        return change_log

    def _update_last_fault(self):
        if self.last_updated_fault is None or not self.last_updated_fault.is_faulted:
            faulted_devices = list(
                filter(lambda f: f.is_faulted and not f.is_communication_channel, self.fault_definitions))
            if len(faulted_devices) > 0:
                self.last_updated_fault = faulted_devices[0]

    def _update_phase(self, readable: Readable) -> LogMessage:
        old_phase_id = self.current_phase_id
        self.current_phase_id, = struct.unpack('<B', readable.read(1))
        self.current_phase = read_string(readable)
        return LogMessage(LogLevel.SYSTEM,
                          f'Phase{'' if old_phase_id < 0 else ' changed'}: {self.current_phase} ({self.current_phase_id})')
