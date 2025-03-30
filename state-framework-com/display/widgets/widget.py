from abc import abstractmethod

from asciimatics.screen import Screen

class Widget:
    screen: Screen
    offset_x: int
    offset_y: int
    width: int
    height: int

    def __init__(self, screen: Screen, offset_x: int, offset_y: int, width: int, height: int):
        self.screen = screen
        self.offset_x = offset_x
        self.offset_y = offset_y
        self.width = width
        self.height = height

    @abstractmethod
    def render(self) -> None:
        raise NotImplementedError()