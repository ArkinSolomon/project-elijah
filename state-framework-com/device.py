import struct
import time
from typing import Any, List

import serial
from serial.serialutil import SerialException

from framework.log_message import LogMessage, LogLevel
from framework.readable.readable_serial import ReadableSerial
from framework.registered_command import CommandInputType
from framework.state_framework import StateFramework
from framework.time_helper import encode_time

FRAMEWORK_TAG = 0x11335577AAEEFF33
MAX_PACKETS_PER_UPDATE = 1024


class Device:
    last_known_port: str
    tty: ReadableSerial | None = None

    framework_request_sent: bool
    current_reading_framework_tag: bytearray
    uses_state_framework: bool
    is_connected: bool
    state_framework: StateFramework | None

    all_devices: List['Device']

    logs: List[LogMessage]

    def __init__(self, port_path: str, all_devices: List['Device']):
        self.tty = None

        self.framework_request_sent = False
        self.current_reading_framework_tag = bytearray()
        self.uses_state_framework = False
        self.is_connected = False
        self.state_framework = None

        self.all_devices = all_devices
        self.logs = []
        self.connect(port_path)

    def connect(self, port_path: str):
        if self.is_connected:
            return

        self.sys_log(f'Device connected to port {port_path}')
        self.last_known_port = port_path
        self.tty = ReadableSerial(serial.Serial(port_path))
        self.is_connected = True
        self.get_metadata()

    def disconnect(self):
        self.tty = None
        self.framework_request_sent = False
        self.is_connected = False
        self.current_reading_framework_tag = bytearray()
        self.sys_log(f'Device disconnected from port {self.last_known_port}')

    def get_metadata(self):
        if not self.is_connected:
            return

        try:
            # TODO write again after a while
            if not self.framework_request_sent:
                self.tty.write(b'\01')
                self.logs.append(LogMessage(LogLevel.SYSTEM, 'Requesting framework metadata...'))
                self.framework_request_sent = True

            if self.tty.bytes_avail() > 0:
                self.current_reading_framework_tag.append(self.tty.read(1)[0])

            if len(self.current_reading_framework_tag) > 8:
                self.current_reading_framework_tag.pop(0)
            elif len(self.current_reading_framework_tag) < 8:
                return

            tag, = struct.unpack('<Q', self.current_reading_framework_tag)
            if tag != FRAMEWORK_TAG:
                return

            self.state_framework = StateFramework.generate_framework_configuration(self.tty)
            self.uses_state_framework = True
            self.sys_log(f'Parsed framework configuration for {self.state_framework.application_name}')

        except Exception as e:
            self.sys_log(f'Error parsing framework configuration: {e}')
            self.disconnect()
            raise e

    def execute_command(self, command_id: int, data: Any = None):
        if not self.is_connected or not self.uses_state_framework:
            return

        self.sys_log(f"Sending command {command_id}, data: {data}")
        assert self.tty
        assert self.state_framework
        self.tty.write(struct.pack('<B', command_id))

        matching_commands = list(filter(lambda c: c.command_id == command_id, self.state_framework.commands))
        if len(matching_commands) == 0:
            self.sys_log(f"Command {command_id} not found")
            return

        command = matching_commands[0]
        match command.command_input:
            case CommandInputType.TIME:
                curr_time = time.localtime()
                encoded_time = encode_time(curr_time)
                self.tty.write(encoded_time)
            case CommandInputType.DOUBLE:
                encoded_data = struct.pack('<d', float(data))
                self.tty.write(encoded_data)
            case CommandInputType.STRING | CommandInputType.ALPHANUMERIC:
                self.tty.write(struct.pack('<H', len(data)))
                encoded_str = str.encode(data, encoding='utf-8')
                self.tty.write(encoded_str)
            case _:
                pass

    def update(self):
        if not self.is_connected or not self.uses_state_framework:
            self.get_metadata()
            return

        try:
            packets, state_changed, logs = self.state_framework.update(self.tty, MAX_PACKETS_PER_UPDATE)
            self.logs += logs
            self.logs = self.logs[-8192:]
            # if state_changed:
            #     for var_def in self.state_framework.variable_definitions:
            #         if var_def.is_hidden:
            #             continue
            #         elif var_def.data_type == DataType.TIME:
            #             if self.state_framework.state[var_def.variable_id] is not None:
            #                 time_str = time.strftime('%Y-%m-%d %H:%M:%S', self.state_framework.state[var_def.variable_id])
            #                 print(
            #                     f'{var_def.display_name} = {time_str}', end=', ')
            #             else:
            #                 print(f'{var_def.display_name} = Unknown', end = ', ')
            #         else:
            #             print(
            #                 f'{var_def.display_name} = {self.state_framework.state[var_def.variable_id]} {var_def.display_unit}',
            #                 end=', ')
            #     print('')
        except SerialException as e:
            print(e)
            self.disconnect()

    def sys_log(self, message: str) -> None:
        self.logs.append(LogMessage(LogLevel.SYSTEM, message))
