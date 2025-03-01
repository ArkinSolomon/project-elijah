import asyncio
import os
import sys
from asyncio import AbstractEventLoop
from time import sleep

from asciimatics.exceptions import ResizeScreenError
from asciimatics.scene import Scene
from asciimatics.screen import Screen

from device import Device
from home_view import HomeView

devices: [Device] = []

user_has_quit = False


def discover_devices():
    serial_ports = [device for device in os.listdir('/dev') if 'tty.usbmodem' in device]

    removal_devices: [Device] = []
    for device in devices:
        if device.last_known_port not in serial_ports:
            device.disconnect()
            if not device.uses_state_framework:
                removal_devices.append(device)

    for device in removal_devices:
        devices.remove(device)

    for port in serial_ports:
        for device in devices:
            if device.last_known_port == port:
                device.connect(port)
                break
        else:
            devices.append(Device(port, devices))


# Define the scene that you'd like to play.
screen = Screen.open()
s = Scene([HomeView(screen, devices)], -1)
screen.set_scenes([s], s)

while True:
    screen.draw_next_frame()

    if screen.has_resized():
        screen.close()
        screen = Screen.open()
        screen.set_scenes([s], s)


screen.close()