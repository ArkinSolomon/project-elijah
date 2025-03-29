from enum import Enum
from typing import Tuple

from asciimatics.screen import Screen

from display.widgets.widget import Widget


class LineType(Enum):
    VERTICAL = 1
    HORIZONTAL = 2


class LineSegments(Widget):
    # position, start, length, vertical/horizontal
    lines: list[Tuple[int, int, int, LineType]]

    def __init__(self, screen: Screen, offset_x: int, offset_y: int, width: int, height: int):
        super(LineSegments, self).__init__(screen, offset_x, offset_y, width, height)
        self.lines = []

    def add_vertical_line(self, x: int, start_y: int, length: int):
        if x < 0 or start_y < 0 or length <= 0:
            raise ValueError("x, start_y must be >= 0 and length > 0")
        self.lines.append((x, start_y, length, LineType.VERTICAL))

    def add_horizontal_line(self, y: int, start_x: int, length: int):
        if y < 0 or start_x < 0 or length <= 0:
            raise ValueError("y, start_x must be >= 0 and length > 0")
        self.lines.append((y, start_x, length, LineType.HORIZONTAL))

    def render(self) -> None:

        # x, y, intersection character
        intersections: list[Tuple[int, int, str]] = []
        for x, start_y, vert_length, _ in filter(lambda line: line[3] == LineType.VERTICAL, self.lines):
            for y, start_x, horiz_length, _ in filter(lambda line: line[3] == LineType.HORIZONTAL, self.lines):
                if y == start_y and x == start_x:
                    intersections.append((x, y, "┌"))
                elif y == start_y + vert_length - 1 and x == start_x + horiz_length - 1:
                    intersections.append((start_x + horiz_length - 1, y, "┘"))
                elif y == start_y + vert_length - 1 and x == start_x:
                    intersections.append((x, y, "└"))
                elif y == start_y and x == start_x + horiz_length - 1:
                    intersections.append((start_x + horiz_length - 1, y, "┐"))
                elif y == start_y and start_x < x < start_x + horiz_length - 1:
                    intersections.append((x, start_y, "┬"))
                elif y == start_y + vert_length - 1 and start_x < x < start_x + horiz_length - 1:
                    intersections.append((x, start_y + vert_length - 1, "┴"))
                elif x == start_x and start_y < y < start_y + vert_length - 1:
                    intersections.append((start_x, y, "├"))
                elif x == start_x + horiz_length - 1 and start_y < y < start_y + vert_length - 1:
                    intersections.append((start_x + horiz_length - 1, y, "┤"))
                elif start_y < y < start_y + vert_length - 1 and start_x < x < start_x + horiz_length - 1:
                    intersections.append((x, y, "┼"))

        for line in self.lines:
            if line[3] == LineType.VERTICAL:
                x, start_y, vert_length, _ = line
                for y in range(start_y, start_y + vert_length):
                    self.screen.print_at("│", x, y)
            else:
                y, start_x, horiz_length, _ = line
                for x in range(start_x, start_x + horiz_length):
                    self.screen.print_at("─", x, y)

        for intersection in intersections:
            self.screen.print_at(intersection[2], self.offset_x + intersection[0], self.offset_y + intersection[1])
