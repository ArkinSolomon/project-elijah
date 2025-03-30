from enum import Enum


class CommandInputType(Enum):
    NONE = 0
    DOUBLE = 1
    ALPHANUMERIC = 2
    STRING = 3
    TIME = 4

class RegisteredCommand:
    command_id: int
    command_name: str
    input_prompt: str
    command_input: CommandInputType

    is_hidden: bool

    def __init__(self, command_id: int, command_name: str, input_prompt: str, command_input: CommandInputType):
        self.command_id = command_id
        self.command_name = command_name
        self.input_prompt = input_prompt
        self.command_input = command_input

        self.is_hidden = self.command_name == '_'

    def __str__(self) -> str:
        return f'Command {'HIDDEN' if self.is_hidden else self.command_name} ({hex(self.command_id)}) input type: {self.command_input}'