from asciimatics.screen import Screen

from display.widgets.line_segments import LineSegments
from display.widgets.widget import Widget
from framework.fault_definition import FaultDefinition, CommunicationChannel


class FaultsWidget(Widget):
    faults: list[FaultDefinition]
    col_width: int

    def __init__(self, screen: Screen, offset_x: int, offset_y: int, width: int, height: int,
                 faults: list[FaultDefinition]):
        super(FaultsWidget, self).__init__(screen, offset_x, offset_y, width, height)

        self.faults = faults
        self.col_width = (self.width - 4) // 5  # 4 vertical lines, 5 columns

    def render(self) -> None:
        table_lines = LineSegments(self.screen, self.offset_x, self.offset_y, self.width, self.height)
        table_lines.add_horizontal_line(1, 0, self.width)

        for i, comm_channel in enumerate(CommunicationChannel):
            table_lines.add_vertical_line((i + 1) * self.col_width + i, 0, self.height)
            ch_off_x = i * self.col_width + i

            ch_faults = [fault for fault in self.faults if fault.communication_channel == comm_channel]
            this_ch_fault_list: list[FaultDefinition] = list(
                filter(lambda fault: fault.is_communication_channel, ch_faults))
            if len(self.faults) > 0 and (len(this_ch_fault_list) > 0 or comm_channel == CommunicationChannel.NONE):
                is_ch_faulted = comm_channel != CommunicationChannel.NONE and this_ch_fault_list[0].is_faulted
                name = this_ch_fault_list[0].fault_name if comm_channel != CommunicationChannel.NONE else "None"
                self._render_fault(ch_off_x, 0, name, is_ch_faulted, True)

                ch_dev_faults = [fault for fault in ch_faults if not fault.is_communication_channel]
                for j, fault in enumerate(ch_dev_faults):
                    self._render_fault(ch_off_x, 2 + j, fault.fault_name, fault.is_faulted, False)
            else:
                self.screen.print_at(comm_channel.name, self.offset_x + ch_off_x, self.offset_y + 0, attr=Screen.A_BOLD)
        table_lines.render()

    def _render_fault(self, rel_row_x: int, rel_row_y: int, name: str, is_faulted: bool, is_header: bool) -> None:
        label_color = Screen.COLOUR_RED if is_faulted else Screen.COLOUR_WHITE
        self.screen.print_at(name, self.offset_x + rel_row_x, self.offset_y + rel_row_y,
                             attr=Screen.A_BOLD if is_header else Screen.A_NORMAL, colour=label_color)

        status_text = 'FAULT' if is_faulted else 'OK'
        status_text_color = Screen.COLOUR_RED if is_faulted else Screen.COLOUR_GREEN
        status_text_rel_off_x = self.col_width - len(status_text)
        self.screen.print_at(status_text, self.offset_x + rel_row_x + status_text_rel_off_x, self.offset_y + rel_row_y,
                             attr=Screen.A_BOLD if is_header else Screen.A_NORMAL, colour=status_text_color)
