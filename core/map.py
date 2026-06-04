"""
map.py

Defines the Map class used by the simulation.

This is the ground truth environment:
- Grid-based (0 = free, 1 = wall)
- Contains start and goal
- Provides utility functions for collision, neighbours, etc.
"""

from typing import List, Tuple
from collections import deque


Position = Tuple[int, int]


class Map:
    def __init__(self, width: int, height: int):
        self.width = width
        self.height = height

        # 0 = free, 1 = wall
        self.grid: List[List[int]] = [
            [1 for _ in range(width)] for _ in range(height)
        ]

        self.start: Position = (0, 0)
        self.goal: Position = (width - 1, height - 1)

    # =========================
    # Basic Utilities
    # =========================

    def in_bounds(self, x: int, y: int) -> bool:
        return 0 <= x < self.width and 0 <= y < self.height

    def is_free(self, x: int, y: int) -> bool:
        if not self.in_bounds(x, y):
            return False
        return self.grid[y][x] == 0

    def set_cell(self, x: int, y: int, value: int):
        """Set cell value (0 = free, 1 = wall)"""
        if self.in_bounds(x, y):
            self.grid[y][x] = value

    def get_cell(self, x: int, y: int) -> int:
        if not self.in_bounds(x, y):
            return 1  # treat out-of-bounds as walls
        return self.grid[y][x]

    # =========================
    # Neighbour Queries
    # =========================
    
    def get_neighbours(self, x: int, y: int) -> List[Position]:
        """4-directional neighbours (grid-based planning)"""
        directions = [
            (1, 0),
            (-1, 0),
            (0, 1),
            (0, -1),
        ]

        neighbours = []
        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            if self.is_free(nx, ny):
                neighbours.append((nx, ny))

        return neighbours


    # =========================
    # Start / Goal
    # =========================

    def set_start(self, pos: Position):
        self.start = pos

    def set_goal(self, pos: Position):
        self.goal = pos

    # =========================
    # Solvability Check
    # =========================

    def is_solvable(self) -> bool:
        """
        Check if there's a path from start to goal using BFS.
        """

        queue = deque()
        visited = set()

        queue.append(self.start)
        visited.add(self.start)

        while queue:
            current = queue.popleft()

            if current == self.goal:
                return True

            for neighbour in self.get_neighbours(*current):
                if neighbour not in visited:
                    visited.add(neighbour)
                    queue.append(neighbour)

        return False

    # =========================
    # Utility Helpers
    # =========================

    def find_random_free(self, rng) -> Position:
      """
      Find a random free cell NOT on the boundary.
      """

      free_cells = [
          (x, y)
          for y in range(1, self.height - 1)
          for x in range(1, self.width - 1)
          if self.grid[y][x] == 0
      ]

      if not free_cells:
          raise ValueError("No valid interior free cells available")

      choice = rng.choice(free_cells)
      return choice if isinstance(choice, tuple) else tuple(choice)

    def clone(self):
        """Deep copy of the map"""
        new_map = Map(self.width, self.height)
        new_map.grid = [row[:] for row in self.grid]
        new_map.start = self.start
        new_map.goal = self.goal
        return new_map

    # =========================
    # Debug / Display
    # =========================

    def __str__(self):
        symbols = {
            0: ".",
            1: "#"
        }

        rows = []
        for y in range(self.height):
            row = ""
            for x in range(self.width):

                if (x, y) == self.start:
                    row += "S"
                elif (x, y) == self.goal:
                    row += "G"
                else:
                    row += symbols[self.grid[y][x]]

            rows.append(row)

        return "\n".join(rows)