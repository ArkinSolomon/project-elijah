import textwrap
from typing import Callable, Tuple

from asciimatics.screen import Screen

from display.color_manager import color_defs
from display.widgets.widget import Widget
from framework.log_message import LogMessage, LogLevel


class LogWidget(Widget):
    log_lines: list[Tuple[str, int]]
    current_scroll_position: int
    on_scroll_position_change: Callable[[int], None] | None
    background_color: int

    def __init__(self, screen: Screen, offset_x: int, offset_y: int, width: int, height: int,
                 logs: list[LogMessage], current_scroll_position: int,
                 on_scroll_position_change: Callable[[int], None] | None, background_color: int):
        super(LogWidget, self).__init__(screen, offset_x, offset_y, width, height)

        self.current_scroll_position = current_scroll_position
        self.on_scroll_position_change = on_scroll_position_change
        self.background_color = background_color

        self._create_log_lines(logs)

    def handle_char(self, char: int) -> bool:
        if char == -204:
            if self.current_scroll_position < 0:
                if len(self.log_lines) <= self.height:
                    self.current_scroll_position = 0
                else:
                    self.current_scroll_position = len(self.log_lines) - self.height
            else:
                self.current_scroll_position -= 1

            if self.current_scroll_position < 0:
                self.current_scroll_position = 0

            if self.on_scroll_position_change is not None:
                self.on_scroll_position_change(self.current_scroll_position)
            return True
        elif char == -206:
            if self.current_scroll_position < 0:
                return True

            if self.current_scroll_position + self.height >= len(self.log_lines) - 1:
                self.current_scroll_position = -1
            else:
                self.current_scroll_position += 1

            if self.on_scroll_position_change is not None:
                self.on_scroll_position_change(self.current_scroll_position)
            return True
        elif char == 32:
            if self.current_scroll_position < 0:
                return True

            self.current_scroll_position = -1
            if self.on_scroll_position_change is not None:
                self.on_scroll_position_change(self.current_scroll_position)
            return True
        return False

    def render(self) -> None:
        if self.current_scroll_position < 0:
            if len(self.log_lines) < self.height:
                print_lines = self.log_lines
            else:
                print_lines = self.log_lines[(len(self.log_lines) - self.height):]
        else:
            print_lines = self.log_lines[self.current_scroll_position:self.current_scroll_position + self.height]

        for i, line in enumerate(print_lines):
            message_line, color = line
            self.screen.print_at(message_line, self.offset_x, i + self.offset_y, colour=color, bg=self.background_color)

    def _create_log_lines(self, logs: list[LogMessage]) -> None:
        self.log_lines = []
        for log in logs:
            header = log.get_log_header_str()
            header_len = len(header)

            color = self._get_level_color(log.log_level)

            wrapped = ['\n'.join(textwrap.wrap(line, self.width, initial_indent=header, break_long_words=False,
                                               replace_whitespace=False))
                       for line in log.message.splitlines()]

            if len(wrapped) == 0:
                continue
            self.log_lines.append((wrapped[0], color))
            for i in range(1, len(wrapped)):
                self.log_lines.append(((" " * (header_len - 2)) + ': ' + wrapped[i][header_len:], color))

    def _get_level_color(self, level: LogLevel) -> int:
        match level:
            case LogLevel.SERIAL_ONLY:
                return color_defs.serial_log
            case LogLevel.DEBUG:
                return color_defs.debug_log
            case LogLevel.INFO:
                return color_defs.info_log
            case LogLevel.WARNING:
                return color_defs.warning_log
            case LogLevel.ERROR:
                return color_defs.error_log
            case LogLevel.SYSTEM:
                return color_defs.system_log
            case _:
                raise Exception(f'No color for LogLevel: {level}')