import struct
from enum import Enum

import serial
from serial.serialutil import SerialException

from framework.state_framework import StateFramework
from serial_helper import read_string, read_fixed_string

MAX_PACKETS_PER_UPDATE = 1024
FRAMEWORK_TAG = 0xBC7AA65201C73901


class OutputPacket(Enum):
    LOG_MESSAGE = 1
    STATE_UPDATE = 2
    PERSISTENT_STATE_UPDATE = 3


class LogLevel(Enum):
    DEBUG = 0
    DEFAULT = 1
    WARNING = 2
    ERROR = 3


class Device:
    last_known_port: str
    tty: serial.Serial | None = None

    uses_state_framework = False
    is_connected = False
    state_framework: StateFramework | None = None

    all_devices: ['Device']

    def __init__(self, port_path: str, all_devices: ['Device']):
        self.all_devices = all_devices
        self.connect(port_path)

    def connect(self, port_path: str):
        if self.is_connected:
            return

        self.last_known_port = port_path
        self.tty = serial.Serial(port_path)
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
            while self.tty.in_waiting < 8:
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

    def update(self):
        if not self.is_connected or not self.uses_state_framework:
            self.get_metadata()
            return

        packets_read = 0
        while packets_read < MAX_PACKETS_PER_UPDATE:
            if self.tty.in_waiting == 0:
                break

            try:
                packet_id, = struct.unpack('<B', self.tty.read(1))
                output_packet = OutputPacket(packet_id)
                packets_read += 1

                match output_packet:
                    case OutputPacket.LOG_MESSAGE:
                        log_level_id, message_length = struct.unpack('<BH', self.tty.read(3))
                        log_level = LogLevel(log_level_id)
                        log_message = read_fixed_string(self.tty, message_length)
                        self.log(log_level, log_message)
                    case OutputPacket.STATE_UPDATE:
                        self.state_framework.state_updated(self.tty)
                        print(self.state_framework.state)
                    case _:
                        print(f'Unknown output packet: {output_packet} ({packet_id})')
            except SerialException as e:
                print(e)
                self.disconnect()
            except Exception as e:
                continue

    @staticmethod
    def log(log_level: LogLevel, message: str):
        match log_level:
            case LogLevel.DEBUG:
                prefix = 'DEBUG: '
            case LogLevel.WARNING:
                prefix = 'WARNING: '
            case LogLevel.ERROR:
                prefix = 'ERROR: '
            case _:
                prefix = 'INFO: '
        print(f'{prefix}{message}')