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
                print(f"Start/Goal placed successfully at attempt {attempt + 1}")
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