from typing import Callable

from asciimatics.screen import Screen

from display.color_manager import color_defs
from display.widgets.line_segments import LineSegments
from display.widgets.widget import Widget
from framework.fault_definition import FaultDefinition, CommunicationChannel


class FaultsWidget(Widget):
    faults: list[FaultDefinition]
    last_updated_fault: FaultDefinition | None
    col_width: int

    is_selected: int
    selected_col: int
    selected_row: int
    on_select_change: Callable[[int, int], None] | None

    background_color: int

    row_counts: list[int]

    def __init__(self, screen: Screen, offset_x: int, offset_y: int, width: int, height: int,
                 faults: list[FaultDefinition], last_updated_fault: FaultDefinition | None, is_selected: bool,
                 selected_col: int,
                 selected_row: int, on_select_change: Callable[[int, int], None] | None, background_color: int):
        super(FaultsWidget, self).__init__(screen, offset_x, offset_y, width, height)

        self.faults = faults
        self.last_updated_fault = last_updated_fault
        self.col_width = (self.width - 4) // 5  # 4 vertical lines, 5 columns

        self.is_selected = is_selected
        self.selected_col = selected_col
        self.selected_row = selected_row
        self.on_select_change = on_select_change
        self.background_color = background_color

        self.row_counts: list[int] = []
        for com_channel in CommunicationChannel:
            self.row_counts.append(len(list(
                filter(lambda f: not f.is_communication_channel and f.communication_channel == com_channel, faults))))

    def handle_char(self, char: int) -> bool:
        if char == -203:
            self.selected_col -= 1
            if self.selected_col < 0:
                self.selected_col = 0

            if self.row_counts[self.selected_col] == 0:
                if self.selected_col == 0:
                    return self.handle_char(-205)
                else:
                    return self.handle_char(-203)

            if self.selected_row >= self.row_counts[self.selected_col]:
                self.selected_row = self.row_counts[self.selected_col] - 1

            if self.on_select_change is not None:
                self.on_select_change(self.selected_col, self.selected_row)
            return True
        elif char == -205:
            self.selected_col += 1
            if self.selected_col >= len(self.row_counts):
                self.selected_col = len(self.row_counts) - 1

            if self.row_counts[self.selected_col] == 0:
                if self.selected_col == len(self.row_counts) - 1:
                    return self.handle_char(-203)
                else:
                    return self.handle_char(-205)

            if self.selected_row >= self.row_counts[self.selected_col]:
                self.selected_row = self.row_counts[self.selected_col] - 1

            if self.on_select_change is not None:
                self.on_select_change(self.selected_col, self.selected_row)
            return True
        elif char == -204:
            self.selected_row -= 1
            if self.selected_row < 0:
                self.selected_row = 0

            if self.on_select_change is not None:
                self.on_select_change(self.selected_col, self.selected_row)
            return True
        elif char == -206:
            self.selected_row += 1
            if self.selected_row >= self.row_counts[self.selected_col]:
                self.selected_row = self.row_counts[self.selected_col] - 1

            if self.on_select_change is not None:
                self.on_select_change(self.selected_col, self.selected_row)
            return True
        return False

    def render(self) -> None:
        table_lines = LineSegments(self.screen, self.offset_x, self.offset_y, self.width, self.height, self.background_color)
        table_lines.add_horizontal_line(1, 0, self.width)
        table_lines.add_horizontal_line(self.height - 2, 0, self.width)

        if not self.is_selected and self.last_updated_fault is not None:
            self._render_fault_reason(self.last_updated_fault)

        for i, comm_channel in enumerate(CommunicationChannel):
            table_lines.add_vertical_line((i + 1) * self.col_width + i, 0, self.height - 1)
            ch_off_x = i * self.col_width + i

            ch_faults = [fault for fault in self.faults if fault.communication_channel == comm_channel]
            this_ch_fault_list: list[FaultDefinition] = list(
                filter(lambda fault: fault.is_communication_channel, ch_faults))
            if len(self.faults) > 0 and (len(this_ch_fault_list) > 0 or comm_channel == CommunicationChannel.NONE):
                ch_dev_faults = [fault for fault in ch_faults if not fault.is_communication_channel]

                if comm_channel == CommunicationChannel.NONE:
                    self.screen.print_at("None", self.offset_x + ch_off_x, self.offset_y + 0, attr=Screen.A_BOLD,
                                         bg=self.background_color)
                else:
                    this_ch_fault_def = this_ch_fault_list[0]
                    is_ch_faulted = this_ch_fault_def.is_faulted
                    name = this_ch_fault_def.fault_name
                    self._render_fault(ch_off_x, 0, name, is_ch_faulted, len(ch_dev_faults) > 0, True, False)

                for j, fault in enumerate(ch_dev_faults):
                    cell_selected = self.is_selected and i == self.selected_col and j == self.selected_row
                    self._render_fault(ch_off_x, 2 + j, fault.fault_name, fault.is_faulted, True, False, cell_selected)
                    if cell_selected:
                        self._render_fault_reason(fault)
            else:
                self.screen.print_at(comm_channel.name, self.offset_x + ch_off_x, self.offset_y + 0, attr=Screen.A_BOLD,
                                     bg=self.background_color)
        table_lines.render()

    def _render_fault_reason(self, fault: FaultDefinition) -> None:
        fault_label = fault.fault_name + ': '
        self.screen.print_at(fault_label, self.offset_x, self.height, attr=Screen.A_BOLD, bg=self.background_color)
        rem_space = self.width - len(fault_label)

        if fault.is_faulted:
            fault_msg = fault.last_fault_message
        else:
            fault_msg = 'Fault resolved!'

        if fault_msg is None:
            fault_msg = 'No fault message'
        elif len(fault_msg) > rem_space:
            fault_msg = fault_msg[:rem_space - 3] + '...'

        self.screen.print_at(fault_msg, self.offset_x + len(fault_label), self.height,
                             colour=Screen.COLOUR_WHITE if fault.is_faulted else Screen.COLOUR_GREEN,
                             bg=self.background_color)

    def _render_fault(self, rel_row_x: int, rel_row_y: int, name: str, is_faulted: bool, fault_known: bool,
                      is_header: bool, row_selected: bool) -> None:
        label_color = Screen.COLOUR_RED if is_faulted else Screen.COLOUR_WHITE
        bg = self.background_color
        if row_selected:
            bg = color_defs.selection
            self.screen.print_at(' ' * self.col_width, self.offset_x + rel_row_x, self.offset_y + rel_row_y, bg=bg)

        self.screen.print_at(name, self.offset_x + rel_row_x, self.offset_y + rel_row_y,
                             attr=Screen.A_BOLD if is_header else Screen.A_NORMAL, colour=label_color,
                             bg=bg)

        status_text = 'FAULT' if is_faulted else 'OK'
        status_text_color = color_defs.bad if is_faulted else color_defs.good
        if not fault_known:
            status_text = 'N/A'
            status_text_color = color_defs.disabled

        status_text_rel_off_x = self.col_width - len(status_text)


        self.screen.print_at(status_text, self.offset_x + rel_row_x + status_text_rel_off_x, self.offset_y + rel_row_y,
                             attr=Screen.A_BOLD if is_header else Screen.A_NORMAL, colour=status_text_color,
                             bg=bg)
