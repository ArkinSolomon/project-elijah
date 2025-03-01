import struct

import serial

from framework.framework_metadata import FrameworkMetadata

framework_tag = 0xBC7AA65201C73901

class Device:
    last_known_port: str
    tty: serial.Serial | None = None

    uses_state_framework = False
    is_connected = False
    framework_metadata: FrameworkMetadata | None = None

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
            self.tty.write(b'\01')
            if self.tty.in_waiting < 8:
                return
            read_data = self.tty.read(8)

            tag, = struct.unpack('<Q', read_data)

            if tag == framework_tag:
                self.framework_metadata = FrameworkMetadata.generate_framework_configuration(self.tty)
                self.uses_state_framework = True
                print(f'Parsed framework configuration for {self.framework_metadata.application_name}')

        except Exception as e:
            print(e)
            self.disconnect()

    def update(self):
        if not self.is_connected or not self.uses_state_framework:
            self.get_metadata()
            return
