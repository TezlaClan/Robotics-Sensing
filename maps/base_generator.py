"""
base_generator.py

Defines the base interface for all map generators.

All generators must:
- Use the provided RandomManager
- Generate a valid Map object
- Ensure the map is solvable
"""

from abc import ABC, abstractmethod
from typing import Optional

from core.map import Map
from utils.random_manager import RandomManager
from utils.debug import dprint


class BaseMapGenerator(ABC):
    def __init__(self, width: int, height: int, rng_manager: RandomManager):
        self.width = width
        self.height = height
        self.rng_manager = rng_manager
        self.rng = rng_manager.map_rng()

    # =========================
    # Public API
    # =========================

    def generate(self) -> Map:
        """
        Main entry point.

        Handles:
        - Generation
        - Start/goal placement
        - Solvability validation
        """

        max_attempts = 10

        for attempt in range(max_attempts):
            map_obj = self._generate_map()

            self._place_start_and_goal(map_obj)

            if map_obj.is_solvable():
                return map_obj

        raise RuntimeError("Failed to generate a solvable map after multiple attempts")

    # =========================
    # Abstract methods
    # =========================

    @abstractmethod
    def _generate_map(self) -> Map:
        """
        Must:
        - Create a Map object
        - Fill grid (0 = free, 1 = wall)
        """
        pass

    # =========================
    # Helper Methods
    # =========================
  
    def _place_start_and_goal(self, map_obj: Map):
        tries = 1000
        
        start_failures = 0
        goal_failures = 0
        same_pos_failures = 0
        clearance_failures = 0

        for attempt in range(tries):
            start = map_obj.find_random_free(self.rng)
            goal = map_obj.find_random_free(self.rng)

            if start == goal:
                same_pos_failures += 1
                continue

            start_clear = self._has_clearance(map_obj, start)
            goal_clear = self._has_clearance(map_obj, goal)
            
            if not start_clear:
                start_failures += 1
            if not goal_clear:
                goal_failures += 1
            
            if start_clear and goal_clear:
                map_obj.set_start(start)
                map_obj.set_goal(goal)
                dprint(f"Start/Goal placed successfully at attempt {attempt + 1}")
                return
            else:
                clearance_failures += 1

        raise RuntimeError(
            f"Failed to place start/goal after {tries} attempts:\n"
            f"  - Same position: {same_pos_failures}\n"
            f"  - Start not clear: {start_failures}\n"
            f"  - Goal not clear: {goal_failures}\n"
            f"  - Clearance check failed: {clearance_failures}"
        )
    
    
    def _has_clearance(self, map_obj, pos):
        """
        ⚠️ IMPORTANT: Do NOT add any clearance checks here!
        This is a tight maze - any neighbor checks will cause failures.
        Just check if the cell itself is free.
        The +0.5 offset in main.py handles centering the agent in the tile.
        """
        x, y = pos
        # Just check if the cell is free, that's it
        return map_obj.get_cell(x, y) == 0


    # =========================
    # Utility Helpers for subclasses
    # =========================

    def _fill_map(self, map_obj: Map, value: int):
        """Fill entire map with value (0 or 1)"""
        for y in range(map_obj.height):
            for x in range(map_obj.width):
                map_obj.grid[y][x] = value

    def _random_wall_density(self, map_obj: Map, density: float):
        """
        Fill map randomly with walls based on density.

        density:
            0.0 = all free
            1.0 = all walls
        """
        for y in range(map_obj.height):
            for x in range(map_obj.width):
                if self.rng.random() < density:
                    map_obj.grid[y][x] = 1
                else:
                    map_obj.grid[y][x] = 0

    def _carve_rectangle(self, map_obj: Map, x1: int, y1: int, x2: int, y2: int):
        """Carve out a rectangular room (set to free space)"""
        for y in range(max(0, y1), min(map_obj.height, y2)):
            for x in range(max(0, x1), min(map_obj.width, x2)):
                map_obj.grid[y][x] = 0

    def _keep_largest_region(self, map_obj: Map):
        """
        Flood-fill the free space and keep only the largest connected region,
        walling off all smaller pockets. Guarantees the map is fully connected
        (and therefore solvable) - useful for generators that can leave isolated
        cavities, e.g. caves and scattered-obstacle fields.
        """
        from collections import deque

        visited = [[False] * map_obj.width for _ in range(map_obj.height)]
        largest = []

        for sy in range(map_obj.height):
            for sx in range(map_obj.width):
                if map_obj.grid[sy][sx] != 0 or visited[sy][sx]:
                    continue

                # Flood-fill this connected free region
                region = []
                queue = deque([(sx, sy)])
                visited[sy][sx] = True

                while queue:
                    x, y = queue.popleft()
                    region.append((x, y))

                    for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                        nx, ny = x + dx, y + dy
                        if (map_obj.in_bounds(nx, ny)
                                and not visited[ny][nx]
                                and map_obj.grid[ny][nx] == 0):
                            visited[ny][nx] = True
                            queue.append((nx, ny))

                if len(region) > len(largest):
                    largest = region

        keep = set(largest)
        for y in range(map_obj.height):
            for x in range(map_obj.width):
                if map_obj.grid[y][x] == 0 and (x, y) not in keep:
                    map_obj.set_cell(x, y, 1)