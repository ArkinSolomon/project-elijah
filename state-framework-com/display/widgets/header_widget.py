from dataclasses import dataclass
from datetime import datetime
from typing import Any, Callable, Tuple

from asciimatics.screen import Screen

from display.widgets.widget import Widget
from framework.data_type import DataType
from framework.persistent_data_entry import PersistentDataEntry
from framework.variable_definition import VariableDefinition


@dataclass
class HeaderColumn:
    offset_x: int
    width: int
    data: list[Tuple[str, str]]  # (label, data+unit)


class HeaderWidget(Widget):
    app_name: str
    state: dict[int, Any]
    variable_definitions: list[VariableDefinition]
    persistent_entries: list[PersistentDataEntry]

    on_page_change: Callable[[int], None] | None
    current_page: int
    background_color: int

    row_count: int
    pages: list[list[HeaderColumn]]

    def __init__(self, screen: Screen, offset_x: int, offset_y: int, width: int, height: int, app_name: str,
                 state: dict[int, Any], variable_definitions: list[VariableDefinition],
                 persistent_entries: list[PersistentDataEntry], current_page: int,
                 on_page_change: Callable[[int], None] | None, background_color: int):
        super(HeaderWidget, self).__init__(screen, offset_x, offset_y, width, height)

        self.app_name = app_name

        self.state = state
        self.variable_definitions = variable_definitions
        self.persistent_entries = persistent_entries
        self.current_page = current_page
        self.on_page_change = on_page_change
        self.background_color = background_color

        self.row_count = self.height - 1
        self.pages = []
        self.gen_rows()

    def handle_char(self, char: int) -> bool:
        return False

    def render(self) -> None:
        curr_time_str = datetime.now().strftime("%A %B %-d, %Y %H:%M:%S")
        self.screen.print_at(curr_time_str + f' | {self.app_name}', self.offset_x, self.offset_y, bg=self.background_color, attr=Screen.A_BOLD)

        if len(self.pages) == 0:
            return

        disp_page = self.pages[self.current_page]
        for column in disp_page:
            for i, row in enumerate(column.data):
                label, value = row
                self.screen.print_at(label, self.offset_x + column.offset_x, i + 1 + self.offset_y, bg=self.background_color)
                self.screen.print_at(value, self.offset_x + column.offset_x + (column.width - len(value)), i + 1 + self.offset_y, colour=Screen.COLOUR_CYAN,  bg=self.background_color)


    def gen_rows(self) -> None:
        page_offset = 0
        curr_page: list[HeaderColumn] = []
        curr_data_column: list[Tuple[str, str]] = []
        for i, var_def in enumerate(self.variable_definitions):
            if var_def.is_hidden:
                continue

            unit = '%' if var_def.display_unit == '%' else f' {var_def.display_unit}' if var_def.display_unit != '' else ''
            if var_def.data_type == DataType.DOUBLE:
                curr_data_column.append(
                    (var_def.display_name + ':', f'{self.state[var_def.variable_id]:.3f}{unit}'))
            elif var_def.data_type == DataType.TIME:
                curr_data_column.append(
                    (var_def.display_name + ':', self.state[var_def.variable_id].strftime("%A %B %-d, %Y %H:%M:%S"))
                )
            else:
                curr_data_column.append(
                    (var_def.display_name + ':', f'{self.state[var_def.variable_id]}{unit}'))

            if len(curr_data_column) == self.row_count or i == len(self.variable_definitions) - 1:
                max_name_width = max(len(display_name) + 1 for display_name, _ in curr_data_column)
                max_val_width = max(len(display_value) for _, display_value in curr_data_column)

                col_width = max_name_width + max_val_width

                if page_offset + col_width > self.width:
                    self.pages.append(curr_page)
                    page_offset = 0
                    curr_page = []

                curr_page.append(HeaderColumn(page_offset, col_width, curr_data_column))
                page_offset += col_width + 1
                curr_data_column = []

        self.pages.append(curr_page)
