import struct

import serial

framework_tag = 0x61726B696E736F6C

class Device:
    last_known_port: str
    tty: serial.Serial | None = None

    uses_state_framework = False
    is_connected = False

    application_name: str | None = None
    all_devices: ['Device']

    def __init__(self, port_path: str, all_devices: ['Device']):
        self.last_known_port = port_path
        self.all_devices = all_devices

    def connect(self, port_path: str):
        if self.is_connected:
            return

        self.last_known_port = port_path
        self.tty = serial.Serial(port_path, 115200, timeout=1)
        self.is_connected = True

    def disconnect(self):
        self.tty = None
        self.is_connected = False

    def scan_for_framework_tag(self):
        if not self.is_connected:
            return

        read_data = self.tty.read(8)
        tag, = struct.unpack('<Q', read_data)
        if tag == framework_tag:
            self.uses_state_framework = True

    def update(self):
        if not self.uses_state_framework:
            self.scan_for_framework_tag()
            return

