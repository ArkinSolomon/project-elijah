import re
from typing import Callable
import platform

from asciimatics.screen import Screen

from device import Device
from display.color_manager import color_defs
from display.screen_state import ScreenState, InputMode, ScreenWidget
from display.widgets.color_block import ColorBlock
from display.widgets.faults_widget import FaultsWidget
from display.widgets.header_widget import HeaderWidget
from display.widgets.line_segments import LineSegments
from display.widgets.list_widget import ListWidget
from display.widgets.log_widget import LogWidget
from framework.registered_command import CommandInputType

HEADER_SIZE = 8
FAULT_WIDTH = 19 * 5 + 4
SIDEBAR_WIDTH = 25
DEVICE_SELECTION_HEIGHT = 5


def header_page_changed(screen_state: ScreenState) -> Callable[[int], None]:
    def update(page: int) -> None:
        screen_state.current_header_page = page

    return update


def fault_sel_changed(screen_state: ScreenState) -> Callable[[int, int], None]:
    def update(col: int, row: int) -> None:
        screen_state.current_fault_col = col
        screen_state.current_fault_row = row

    return update


def log_scroll_changed(screen_state: ScreenState) -> Callable[[int], None]:
    def update(pos: int) -> None:
        screen_state.current_scroll_pos = pos

    return update


def update_command_idx(screen_state: ScreenState) -> Callable[[int], None]:
    def update(command_idx: int) -> None:
        screen_state.selected_command_idx = command_idx

    return update


def update_hovered_device_idx(screen_state: ScreenState) -> Callable[[int], None]:
    def update(hovered_device_idx: int) -> None:
        screen_state.hovered_device_idx = hovered_device_idx

    return update


def handle_device_switch(screen_state: ScreenState) -> Callable[[int], None]:
    def update(selected_idx: int) -> None:
        screen_state.selected_device_idx = selected_idx
        screen_state.current_fault_col = 0
        screen_state.current_fault_row = 0

    return update


def handle_command(screen_state: ScreenState, device: Device | None) -> Callable[[int], None]:
    if device is None:
        def handle_empty(_: int): pass

        return handle_empty

    def handle(command_idx: int):
        assert device.state_framework
        shown_commands = [command for command in device.state_framework.commands if
                          not command.command_name.startswith('_')]
        command = shown_commands[command_idx]
        if command.command_input == CommandInputType.NONE or command.command_input == CommandInputType.TIME:
            device.execute_command(command.command_id)
            return

        match command.command_input:
            case CommandInputType.DOUBLE:
                screen_state.input_mode = InputMode.DOUBLE
            case CommandInputType.ALPHANUMERIC:
                screen_state.input_mode = InputMode.ALPHA_NUMERIC
            case CommandInputType.STRING:
                screen_state.input_mode = InputMode.STRING

        screen_state.input_prompt = command.input_prompt

        def enter_callback(input_str: str):
            if input_str == CommandInputType.DOUBLE:
                device.execute_command(command.command_id, float(input_str))
            else:
                device.execute_command(command.command_id, input_str)

        screen_state.enter_callback = enter_callback

    return handle


def screen_loop(screen: Screen, screen_state: ScreenState, devices: list[Device]) -> bool:
    line_segments = LineSegments(screen, 0, 0, screen.width, screen.height, Screen.COLOUR_BLACK)

    line_segments.add_vertical_line(0, 0, screen.height - 1)
    line_segments.add_vertical_line(screen.width - 1, 0, screen.height - 1)
    line_segments.add_horizontal_line(0, 0, screen.width)
    line_segments.add_horizontal_line(screen.height - 2, 0, screen.width)

    line_segments.add_horizontal_line(HEADER_SIZE + 1, 0, screen.width)
    line_segments.add_vertical_line(screen.width - 2 - FAULT_WIDTH, 0, HEADER_SIZE + 2)
    line_segments.add_vertical_line(screen.width - 2 - SIDEBAR_WIDTH, HEADER_SIZE + 1, screen.height - HEADER_SIZE - 2)
    line_segments.add_horizontal_line(screen.height - HEADER_SIZE, screen.width - 2 - SIDEBAR_WIDTH, SIDEBAR_WIDTH + 1)

    line_segments.render()

    char_handled = False
    ev = screen.get_key()
    if ev == 13 and platform.system() == 'Windows':
        ev = 10

    header_selected = screen_state.focused_widget == ScreenWidget.HEADER and screen_state.input_mode != CommandInputType.NONE
    header_background = color_defs.selected_widget_background if header_selected else Screen.COLOUR_BLACK
    header_block = ColorBlock(screen, 1, 1, screen.width - 3 - FAULT_WIDTH, HEADER_SIZE, header_background)
    header_block.render()

    if screen_state.selected_device_idx < len(devices) and devices[
        screen_state.selected_device_idx].uses_state_framework:
        sf = devices[screen_state.selected_device_idx].state_framework
        assert sf is not None
        header_widget = HeaderWidget(screen, 1, 1, screen.width - 3 - FAULT_WIDTH, HEADER_SIZE, sf.application_name,
                                     sf.current_phase,
                                     sf.state, sf.variable_definitions, sf.persistent_entries,
                                     screen_state.current_header_page, header_page_changed(screen_state),
                                     header_background)

        if header_selected and screen_state.input_mode == InputMode.NONE and ev is not None:
            char_handled = header_widget.handle_char(ev)
    else:
        app_name = "<Unknown>"
        flight_phase = "<Unknown>"
        header_widget = HeaderWidget(screen, 1, 1, screen.width - 2, HEADER_SIZE, app_name, flight_phase, {}, [], [],
                                     screen_state.current_header_page, None, header_background)
    header_widget.render()

    if screen_state.selected_device_idx < len(devices) and devices[
        screen_state.selected_device_idx].uses_state_framework:
        sf = devices[screen_state.selected_device_idx].state_framework
        assert sf is not None
        faults = sf.fault_definitions
        last_updated_fault = sf.last_updated_fault
    else:
        faults = []
        last_updated_fault = None

    faults_selected = screen_state.focused_widget == ScreenWidget.FAULTS and screen_state.input_mode != CommandInputType.NONE
    faults_background = color_defs.selected_widget_background if faults_selected else color_defs.background
    faults_block = ColorBlock(screen, screen.width - 1 - FAULT_WIDTH, 1, FAULT_WIDTH, HEADER_SIZE, faults_background)
    faults_block.render()

    faults_widget = FaultsWidget(screen, screen.width - 1 - FAULT_WIDTH, 1, FAULT_WIDTH, HEADER_SIZE, faults,
                                 last_updated_fault, faults_selected, screen_state.current_fault_col,
                                 screen_state.current_fault_row, fault_sel_changed(screen_state), faults_background)
    faults_widget.render()

    if faults_selected and screen_state.input_mode == InputMode.NONE and ev is not None:
        char_handled = faults_widget.handle_char(ev)

    commands_selected = screen_state.focused_widget == ScreenWidget.COMMANDS and screen_state.input_mode != CommandInputType.NONE
    command_background = color_defs.selected_widget_background if commands_selected else color_defs.background
    commands_block = ColorBlock(screen, screen.width - SIDEBAR_WIDTH - 1, HEADER_SIZE + 2, SIDEBAR_WIDTH,
                                screen.height - HEADER_SIZE - 5 - DEVICE_SELECTION_HEIGHT,
                                command_background)
    commands_block.render()
    screen.print_at("Commands", screen.width - SIDEBAR_WIDTH - 1, HEADER_SIZE + 2, colour=color_defs.text,
                    attr=Screen.A_BOLD if commands_selected else Screen.A_NORMAL, bg=command_background)

    logs_selected = screen_state.focused_widget == ScreenWidget.LOGS and screen_state.input_mode != CommandInputType.NONE
    logs_background = color_defs.selected_widget_background if logs_selected else color_defs.background
    logs_block = ColorBlock(screen, 1, HEADER_SIZE + 2, screen.width - SIDEBAR_WIDTH - 3,
                            screen.height - HEADER_SIZE - 4, logs_background)
    logs_block.render()

    logs = devices[screen_state.selected_device_idx].logs if screen_state.selected_device_idx < len(devices) else []
    log_widget = LogWidget(screen, 1, HEADER_SIZE + 2, screen.width - SIDEBAR_WIDTH - 3,
                           screen.height - HEADER_SIZE - 4, logs, screen_state.current_scroll_pos,
                           log_scroll_changed(screen_state), logs_background)

    if logs_selected and screen_state.input_mode == InputMode.NONE and ev is not None:
        char_handled = log_widget.handle_char(ev)

    log_widget.render()

    if len(devices) > 0 and devices[screen_state.selected_device_idx].uses_state_framework:
        assert devices[screen_state.selected_device_idx].state_framework is not None
        list_options = [command.command_name for command in
                        devices[screen_state.selected_device_idx].state_framework.commands if  # type: ignore
                        not command.command_name.startswith('_')]
    else:
        list_options = []
    command_list = ListWidget(screen, screen.width - SIDEBAR_WIDTH - 1, HEADER_SIZE + 3, SIDEBAR_WIDTH,
                              screen.height - HEADER_SIZE - 6 - DEVICE_SELECTION_HEIGHT,
                              list_options, screen_state.selected_command_idx, update_command_idx(screen_state),
                              handle_command(screen_state,
                                             None if screen_state.selected_device_idx >= len(devices) else devices[
                                                 screen_state.selected_device_idx]),
                              command_background)

    if commands_selected and screen_state.input_mode == InputMode.NONE and ev is not None:
        char_handled = command_list.handle_char(ev)

    command_list.render()

    device_selection_selected = screen_state.focused_widget == ScreenWidget.DEVICES
    device_selection_background = color_defs.selected_widget_background if device_selection_selected else color_defs.background
    device_selection_block = ColorBlock(screen, screen.width - SIDEBAR_WIDTH - 1,
                                        screen.height - 2 - DEVICE_SELECTION_HEIGHT, SIDEBAR_WIDTH,
                                        DEVICE_SELECTION_HEIGHT, device_selection_background)
    device_selection_block.render()
    device_list = ListWidget(screen, screen.width - SIDEBAR_WIDTH - 1, screen.height - 2 - DEVICE_SELECTION_HEIGHT,
                             SIDEBAR_WIDTH, DEVICE_SELECTION_HEIGHT,
                             [device.last_known_port for device in devices], screen_state.hovered_device_idx,
                             update_hovered_device_idx(screen_state),
                             handle_device_switch(screen_state),
                             device_selection_background)

    if device_selection_selected and screen_state.input_mode == InputMode.NONE and ev is not None:
        char_handled = device_list.handle_char(ev)

    device_list.render()

    if screen_state.input_mode == InputMode.NONE:
        if ev in (ord('Q'), ord('q')):
            return True
        elif ev == -301:
            screen_state.focus_next()
        screen.print_at(
            f"Project Elijah State Framework Communication — Cedarville University | {screen.width}x{screen.height} | {len(devices)} {"devices" if len(devices) != 1 else "device"} | Screen: {screen_state.screen_refresh_rate} Hz | Press Q to quit",
            0, screen.height - 1, colour=color_defs.footer_text)
    else:
        if (ev == 10 or ev == -1) and not char_handled:
            if ev != -1 and screen_state.enter_callback is not None:
                screen_state.enter_callback(screen_state.current_input)

            screen_state.input_mode = InputMode.NONE
            screen_state.enter_callback = None
            screen_state.current_input = ""
        elif ev == -300:
            if len(screen_state.current_input) > 0:
                screen_state.current_input = screen_state.current_input[:-1]
        elif ev is not None and not char_handled:
            # noinspection PyBroadException
            try:
                new_input = screen_state.current_input + chr(ev)

                match screen_state.input_mode:
                    case InputMode.DOUBLE:
                        if len(new_input) > 0:
                            try:
                                float(new_input)
                            except ValueError:
                                new_input = screen_state.current_input
                    case InputMode.ALPHA_NUMERIC:
                        if not re.match(r'^[\w ]{0,64}$', screen_state.current_input):
                            new_input = screen_state.current_input
                    case _:
                        pass

                screen_state.current_input = new_input
            except:
                pass

        prompt = f"Press ESC to cancel | {screen_state.input_prompt} > "
        screen.print_at(prompt, 0, screen.height - 1, colour=color_defs.prompt_text, bg=color_defs.prompt_background)
        screen.print_at(screen_state.current_input, len(prompt), screen.height - 1, colour=color_defs.prompt_input, bg=color_defs.prompt_background)

        remaining_fill_size = screen.width - len(prompt) - len(screen_state.current_input)
        screen.print_at(" " * remaining_fill_size, len(prompt) + len(screen_state.current_input), screen.height - 1,
                        bg=color_defs.prompt_background)

    return False
