from asciimatics.screen import Screen

from display.widgets.widget import Widget


class ColorBlock(Widget):
    color: int

    def __init__(self, screen: Screen, offset_x: int, offset_y: int, width: int, height: int, color: int):
        super(ColorBlock, self).__init__(screen, offset_x, offset_y, width, height)
        self.color = color

    def render(self) -> None:
        for y in range(self.height):
            self.screen.print_at(" " * self.width, self.offset_x, y + self.offset_y, bg=self.color)