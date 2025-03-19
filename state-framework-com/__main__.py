import os
import platform

import serial.tools.list_ports
from device import Device

devices: [Device] = []

user_has_quit = False

def discover_devices() -> None:
    ports = serial.tools.list_ports.comports()
    if platform.system() == "Windows":
        serial_ports = [port[0] for port in ports]
    else:
        serial_ports = ['/dev/' + device for device in os.listdir('/dev') if 'tty.usbmodem' in device]

    removal_devices: list[Device] = []
    for device in devices:
        if device.last_known_port not in serial_ports:
            if device.uses_state_framework and device.is_connected:
                print('Device that uses state framework disconnected')
            device.disconnect()
            if not device.uses_state_framework:
                removal_devices.append(device)
                print(f'Unknown device disconnected: {device.last_known_port}')

    for device in removal_devices:
        devices.remove(device)

    for port in serial_ports:
        for device in devices:
            if device.last_known_port == port:
                if not device.is_connected:
                    device.connect(port)
                break
        else:
            print('New device at port: ' + port)
            devices.append(Device(port, devices))

while True:
    discover_devices()
    for device in devices:
        device.update()