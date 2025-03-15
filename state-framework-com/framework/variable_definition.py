from framework.data_type import DataType
from typing import Any

class VariableDefinition:
    variable_id: int
    display_name: str
    display_unit: str
    data_offset: int
    data_type: DataType

    is_hidden: bool = False

    def __init__(self, variable_id: int, display_name: str, display_unit: str, data_offset: int, data_type: DataType):
        self.variable_id = variable_id
        self.display_name = display_name
        self.display_unit = display_unit
        self.data_offset = data_offset
        self.data_type = data_type

        self.is_hidden = self.display_name.startswith('_')

    def __str__(self) -> str:
        return f'{self.display_name} ({hex(self.variable_id)}), units: {self.display_unit}, offset {self.data_offset}, datatype: {self.data_type}'