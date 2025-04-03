import platform
from dataclasses import dataclass

from asciimatics.screen import Screen


@dataclass
class ColorManager:
    text: int
    background: int
    selected_widget_background: int
    selection: int
    disabled: int
    prompt_background: int
    prompt_text: int
    prompt_input: int
    footer_text: int
    bad: int
    good: int
    data: int

    serial_log: int
    debug_log: int
    info_log: int
    warning_log: int
    error_log: int
    system_log: int


if platform.system() == "Windows":
    color_defs = ColorManager(Screen.COLOUR_WHITE, Screen.COLOUR_BLACK, Screen.COLOUR_RED, Screen.COLOUR_MAGENTA,
                              Screen.COLOUR_BLUE, Screen.COLOUR_YELLOW, Screen.COLOUR_BLACK, Screen.COLOUR_BLUE,
                              Screen.COLOUR_BLUE, Screen.COLOUR_RED, Screen.COLOUR_GREEN, Screen.COLOUR_CYAN,
                              Screen.COLOUR_CYAN, Screen.COLOUR_MAGENTA, Screen.COLOUR_WHITE, Screen.COLOUR_YELLOW,
                              Screen.COLOUR_RED, Screen.COLOUR_CYAN)
else:
    color_defs = ColorManager(Screen.COLOUR_WHITE, Screen.COLOUR_BLACK, 235, 55, 243, 226, Screen.COLOUR_BLACK,
                              Screen.COLOUR_BLUE, 242, Screen.COLOUR_RED, Screen.COLOUR_GREEN, Screen.COLOUR_CYAN,
                              Screen.COLOUR_CYAN, Screen.COLOUR_MAGENTA, Screen.COLOUR_WHITE, Screen.COLOUR_YELLOW,
                              Screen.COLOUR_RED, 83)
