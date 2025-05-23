from collections.abc import Callable
from enum import Enum, IntEnum
from typing import Any

class ScreenWidget(IntEnum):
    HEADER = 1
    FAULTS = 2
    LOGS = 3
    COMMANDS = 4
    DEVICES = 5

class InputMode(Enum):
    NONE = 0
    DOUBLE = 1
    ALPHA_NUMERIC = 2
    STRING = 3

class ScreenState:
    focused_widget: ScreenWidget

    current_header_page: int
    current_scroll_pos: int
    selected_command_idx: int

    current_fault_col: int
    current_fault_row: int

    hovered_device_idx: int
    selected_device_idx: int

    last_render: float
    screen_refresh_rate: int

    input_mode: InputMode
    input_prompt: str
    current_input: str
    enter_callback: Callable[[str], Any] | None

    def __init__(self) -> None:
        self.focused_widget = ScreenWidget.HEADER

        self.current_header_page = 0
        self.current_scroll_pos = -1
        self.selected_command_idx = 0

        self.current_fault_col = 0
        self.current_fault_row = 0

        self.hovered_device_idx = 0
        self.selected_device_idx = 0

        self.last_render = 0
        self.screen_refresh_rate = 0

        self.input_mode = InputMode.NONE
        self.current_input = ""
        self.input_prompt = ""
        self.enter_callback = None

    def focus_next(self) -> None:
        focused_num = int(self.focused_widget)
        focused_num += 1
        if focused_num > len(ScreenWidget):
            self.focused_widget = ScreenWidget.HEADER
        else:
            self.focused_widget = ScreenWidget(focused_num)