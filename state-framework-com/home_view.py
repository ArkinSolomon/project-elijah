from asciimatics.effects import Clock
from asciimatics.screen import Screen
from asciimatics.widgets import Frame, Layout, Text, Divider

from device import Device


class HomeView(Frame):
    devices: [Device]

    def __init__(self, screen, devices: [Device]):
        super(HomeView, self).__init__(screen, screen.height, screen.width, has_border=False, has_shadow=False, can_scroll=False)
        self.devices = devices

        header = Layout([1])
        self.add_layout(header)

        header.add_widget(Text("Cedarville University -- Project Elijah", "test", disabled=True))
        header.add_widget(Text("NASA University Student Launch Initiative", disabled=True))
        header.add_widget(Divider())

        main_body = Layout([90, 10], fill_frame=True)
        self.add_layout(main_body)

        main_body.add_widget(Text("1"), 0)
        main_body.add_widget(Text("2"), 1)

        self.fix()
