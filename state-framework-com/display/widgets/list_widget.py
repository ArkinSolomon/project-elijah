from collections.abc import Callable

from asciimatics.screen import Screen

from display.color_manager import color_defs
from display.widgets.widget import Widget


class ListWidget(Widget):
    options: list[str]
    selected_option_idx: int
    on_option_change: Callable[[int], None] | None
    on_option_selected: Callable[[int], None] | None
    default_color: int

    def __init__(self, screen: Screen, offset_x: int, offset_y: int, width: int, height: int,
                 options: list[str], selected_option: int, on_option_change: Callable[[int], None] | None,
                 on_option_selected: Callable[[int], None] | None, default_color: int):
        super(ListWidget, self).__init__(screen, offset_x, offset_y, width, height)
        self.options = options
        self.selected_option_idx = selected_option
        self.on_option_change = on_option_change
        self.on_option_selected = on_option_selected
        self.default_color = default_color

    def handle_char(self, char: int) -> bool:
        if char == -204:
            self.selected_option_idx -= 1
            if self.selected_option_idx < 0:
                self.selected_option_idx = len(self.options) - 1
            if self.on_option_change is not None:
                self.on_option_change(self.selected_option_idx)
            return True
        elif char == -206:
            self.selected_option_idx += 1
            if self.selected_option_idx >= len(self.options):
                self.selected_option_idx = 0
            if self.on_option_change is not None:
                self.on_option_change(self.selected_option_idx)
            return True
        elif char == 10:
            if self.on_option_selected is not None:
                self.on_option_selected(self.selected_option_idx)
            return True
        return False

    def render(self):
        for i, option in enumerate(self.options):
            self.screen.print_at(option, self.offset_x, self.offset_y + i,
                                 bg=color_defs.selection if i == self.selected_option_idx else self.default_color)
            if i == self.selected_option_idx:
                remaining_width = self.width - len(option)
                self.screen.print_at(" " * remaining_width, self.offset_x + len(option), self.offset_y + i, bg=color_defs.selection)