import platform
import time
from dataclasses import dataclass
from datetime import datetime
from typing import Any, Callable, Tuple

from asciimatics.screen import Screen

from display.color_manager import color_defs
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
    flight_phase: str
    state: dict[int, Any]
    variable_definitions: list[VariableDefinition]
    persistent_entries: list[PersistentDataEntry]

    on_page_change: Callable[[int], None] | None
    current_page: int
    background_color: int

    row_count: int
    pages: list[list[HeaderColumn]]

    page_offset: int

    def __init__(self, screen: Screen, offset_x: int, offset_y: int, width: int, height: int, app_name: str,
                 flight_phase: str,
                 state: dict[int, Any], variable_definitions: list[VariableDefinition],
                 persistent_entries: list[PersistentDataEntry], current_page: int,
                 on_page_change: Callable[[int], None] | None, background_color: int):
        super(HeaderWidget, self).__init__(screen, offset_x, offset_y, width, height)

        self.app_name = app_name
        self.flight_phase = flight_phase

        self.state = state
        self.variable_definitions = variable_definitions
        self.persistent_entries = persistent_entries
        self.current_page = current_page
        self.on_page_change = on_page_change
        self.background_color = background_color

        self.row_count = self.height - 1
        self.pages = []
        self.page_offset = 0
        self._gen_rows()

    def handle_char(self, char: int) -> bool:
        if char == -203:
            self.current_page -= 1
            if self.current_page < 0:
                self.current_page = 0

            if self.on_page_change is not None:
                self.on_page_change(self.current_page)
            return True
        elif char == -205:
            self.current_page += 1
            if self.current_page >= len(self.pages):
                self.current_page = len(self.pages) - 1

            if self.on_page_change is not None:
                self.on_page_change(self.current_page)
            return True
        return False

    def render(self) -> None:
        if platform.system() == "Windows":
            curr_time_str = datetime.now().strftime("%A %B %#d, %Y %H:%M:%S")
        else:
            curr_time_str = datetime.now().strftime("%A %B %-d, %Y %H:%M:%S")
        self.screen.print_at(curr_time_str + f' | {self.app_name} | {self.flight_phase}', self.offset_x, self.offset_y,
                             bg=self.background_color, attr=Screen.A_BOLD)

        if len(self.pages) == 0:
            return

        disp_page = self.pages[self.current_page]
        for column in disp_page:
            for i, row in enumerate(column.data):
                label, value = row
                self.screen.print_at(label, self.offset_x + column.offset_x, i + 1 + self.offset_y,
                                     bg=self.background_color)
                self.screen.print_at(value, self.offset_x + column.offset_x + (column.width - len(value)),
                                     i + 1 + self.offset_y, colour=color_defs.data, bg=self.background_color)

    def _gen_rows(self) -> None:
        self._gen_page_for([var_def for var_def in self.variable_definitions if not var_def.is_hidden],
                           lambda var_def: (var_def.display_name, self.state[var_def.variable_id], var_def.data_type,
                                            var_def.display_unit))
        self._gen_page_for(self.persistent_entries, lambda pe: (pe.display_name, pe.current_value, pe.data_type, ''))

    def _gen_page_for[T](self, data_list: list[T], extract: Callable[[T], Tuple[str, Any, DataType, str]]) -> None:
        self.page_offset = 0
        curr_page: list[HeaderColumn] = []
        curr_data_column: list[Tuple[str, str]] = []
        for i, data in enumerate(data_list):
            (label, value, data_type, display_unit) = extract(data)

            unit = '%' if display_unit == '%' else f' {display_unit}' if display_unit != '' else ''
            if data_type == DataType.DOUBLE:
                if abs(value) > 99999 or (abs(value) < 0.0001 and value != 0):
                    curr_data_column.append(
                        (label + ':', f'{value:.3e}{unit}'))
                else:
                    curr_data_column.append((label + ':', f'{value:.3f}{unit}'))
            elif data_type == DataType.TIME:
                if value is None:
                    curr_data_column.append(
                        (label + ':', 'Unknown')
                    )
                else:
                    try:
                        curr_data_column.append(
                            (label + ':', time.strftime("%A %B %-d, %Y %H:%M:%S", value))
                        )
                    except ValueError:
                        curr_data_column.append(
                            (label + ':', 'ERROR!')
                        )
            elif data_type == DataType.STRING:
                curr_data_column.append((label + ':', str(value)))
            else:
                curr_data_column.append(
                    (label + ':', f'{value}{unit}'))

            if len(curr_data_column) == self.row_count or i == len(data_list) - 1:
                max_name_width = max(len(display_name) + 1 for display_name, _ in curr_data_column)
                max_val_width = max(len(display_value) for _, display_value in curr_data_column)

                col_width = max_name_width + max_val_width

                if self.page_offset + col_width > self.width:
                    self.pages.append(curr_page)
                    self.page_offset = 0
                    curr_page = []

                curr_page.append(HeaderColumn(self.page_offset, col_width, curr_data_column))
                self.page_offset += col_width + 1
                curr_data_column = []

        self.pages.append(curr_page)
