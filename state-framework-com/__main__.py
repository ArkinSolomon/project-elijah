import os
import platform
import time

from asciimatics.screen import Screen
from display.screen_loop import screen_loop

import serial.tools.list_ports
from device import Device
from display.screen_state import ScreenState

devices: list[Device] = []
screen_state = ScreenState()

def discover_devices() -> None:
    ports = serial.tools.list_ports.comports()
    if platform.system() == "Windows":
        serial_ports = [port[0] for port in ports if not 'bluetooth' in port.description.lower()]
    else:
        serial_ports = ['/dev/' + device for device in os.listdir('/dev') if 'tty.usbmodem' in device]

    removal_devices: list[Device] = []
    for device in devices:
        if device.last_known_port not in serial_ports:
            if device.is_connected:
                device.disconnect()
            if not device.uses_state_framework:
                removal_devices.append(device)

    for device in removal_devices:
        devices.remove(device)

    for port in serial_ports:
        for device in devices:
            if device.last_known_port == port:
                if not device.is_connected:
                    device.connect(port)
                break
        else:
            devices.append(Device(port, devices))

    if 0 < len(devices) <= screen_state.selected_device_idx:
        screen_state.selected_device_idx = len(devices) - 1

user_has_quit = False
def main(screen: Screen):
    global user_has_quit

    while True:
        discover_devices()
        for device in devices:
            device.update()

        if screen.has_resized():
            return

        screen.clear_buffer(Screen.COLOUR_WHITE, Screen.A_NORMAL, Screen.COLOUR_BLACK)

        dt = time.process_time() - screen_state.last_render
        screen_state.screen_refresh_rate = 9999 if dt == 0 else int(1 / dt)
        user_has_quit = screen_loop(screen, screen_state, devices)

        if user_has_quit:
            break

        screen.refresh()
        screen_state.last_render = time.process_time()

while True:
    Screen.wrapper(main)
    if user_has_quit:
        break