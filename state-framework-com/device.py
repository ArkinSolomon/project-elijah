import struct
from typing import List

import serial
from serial.serialutil import SerialException

from framework.readable.readable import Readable
from framework.readable.readable_serial import ReadableSerial
from framework.registered_command import CommandInputType
from framework.state_framework import StateFramework

FRAMEWORK_TAG = 0xBC7AA65201C73901
MAX_PACKETS_PER_UPDATE = 1024

class Device:
    last_known_port: str
    tty: ReadableSerial | None = None

    uses_state_framework = False
    is_connected = False
    state_framework: StateFramework | None = None

    all_devices: List['Device']

    def __init__(self, port_path: str, all_devices: List['Device']):
        self.all_devices = all_devices
        self.connect(port_path)

    def connect(self, port_path: str):
        if self.is_connected:
            return

        self.last_known_port = port_path
        self.tty = ReadableSerial(serial.Serial(port_path))
        self.is_connected = True

        print(f'Connected to {port_path}')

    def disconnect(self):
        self.tty = None
        self.is_connected = False

    def get_metadata(self):
        if not self.is_connected:
            return

        try:
            # TODO write again after a while
            self.tty.write(b'\01')
            while self.tty.bytes_avail() < 8:
                pass
            read_data = self.tty.read(8)

            tag, = struct.unpack('<Q', read_data)

            if tag == FRAMEWORK_TAG:
                self.state_framework = StateFramework.generate_framework_configuration(self.tty)
                self.uses_state_framework = True
                print(f'Parsed framework configuration for {self.state_framework.application_name}')
                
        except Exception as e:
            print(e)
            self.disconnect()
            raise e

    def update(self):
        if not self.is_connected or not self.uses_state_framework:
            self.get_metadata()
            return

        try:
            packets = self.state_framework.update(self.tty, MAX_PACKETS_PER_UPDATE)
            if packets > 0:
                for var_def in self.state_framework.variable_definitions:
                    if var_def.is_hidden:
                        continue

                    print(f'{var_def.display_name} = {self.state_framework.state[var_def.variable_id]} {var_def.display_unit}', end=', ')
                print('')
        except SerialException as e:
            print(e)
            self.disconnect()
