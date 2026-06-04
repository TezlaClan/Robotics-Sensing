"""
maze_generator.py

Generates a classic maze using DFS (recursive backtracking).

Properties:
- Narrow corridors (1 cell width)
- Fully connected
- No open rooms
"""

from typing import Tuple, List

from core.map import Map
from maps.base_generator import BaseMapGenerator


class MazeGenerator(BaseMapGenerator):
    def _generate_map(self) -> Map:
        """
        Generate maze using DFS backtracking.
        """

        # Maze requires odd dimensions for clean walls
        width = self.width if self.width % 2 == 1 else self.width - 1
        height = self.height if self.height % 2 == 1 else self.height - 1

        map_obj = Map(width, height)

        # Start filled with walls
        self._fill_map(map_obj, 1)

        start = (1, 1)
        stack = [start]

        map_obj.set_cell(1, 1, 0)

        while stack:
            x, y = stack[-1]

            neighbours = self._get_unvisited_neighbours(map_obj, x, y)

            if neighbours:
                nx, ny = self.rng.choice(neighbours)

                # Carve path between cells
                wall_x = (x + nx) // 2
                wall_y = (y + ny) // 2

                map_obj.set_cell(nx, ny, 0)
                map_obj.set_cell(wall_x, wall_y, 0)

                stack.append((nx, ny))
            else:
                stack.pop()

        return map_obj

    # =========================
    # Internal helpers
    # =========================

    def _get_unvisited_neighbours(
        self, map_obj: Map, x: int, y: int
    ) -> List[Tuple[int, int]]:
        """
        Returns neighbours 2 steps away that are still walls.
        """

        directions = [
            (2, 0),
            (-2, 0),
            (0, 2),
            (0, -2),
        ]

        neighbours = []

        for dx, dy in directions:
            nx, ny = x + dx, y + dy

            if map_obj.in_bounds(nx, ny) and map_obj.get_cell(nx, ny) == 1:
                neighbours.append((nx, ny))

        return neighbours