"""
cave_generator.py

Generates organic cave systems using cellular automata.

Properties:
- Irregular, natural-looking caverns (no straight walls)
- Open blob-like spaces connected by narrow gaps
- Fully connected (smaller pockets are walled off)
"""

from core.map import Map
from maps.base_generator import BaseMapGenerator


class CaveGenerator(BaseMapGenerator):
    # Tuning
    FILL_PROBABILITY = 0.45   # initial wall chance per interior cell
    SMOOTHING_STEPS = 5       # cellular-automata iterations
    WALL_LIMIT = 5            # >= this many wall neighbours -> becomes wall
    FREE_LIMIT = 3            # <= this many wall neighbours -> becomes free

    def _generate_map(self) -> Map:
        map_obj = Map(self.width, self.height)

        # =========================
        # 1. Random seed fill (borders stay walls)
        # =========================
        for y in range(1, self.height - 1):
            for x in range(1, self.width - 1):
                wall = self.rng.random() < self.FILL_PROBABILITY
                map_obj.set_cell(x, y, 1 if wall else 0)

        # =========================
        # 2. Smooth into caverns
        # =========================
        for _ in range(self.SMOOTHING_STEPS):
            self._smooth(map_obj)

        # =========================
        # 3. Keep a single connected cavern
        # =========================
        self._keep_largest_region(map_obj)

        return map_obj

    # =========================
    # Cellular automata step
    # =========================

    def _smooth(self, map_obj: Map):
        """One smoothing pass, computed against a snapshot of the grid."""
        new_grid = [row[:] for row in map_obj.grid]

        for y in range(1, self.height - 1):
            for x in range(1, self.width - 1):
                walls = self._count_wall_neighbours(map_obj, x, y)

                if walls >= self.WALL_LIMIT:
                    new_grid[y][x] = 1
                elif walls <= self.FREE_LIMIT:
                    new_grid[y][x] = 0
                # otherwise: leave cell unchanged

        map_obj.grid = new_grid

    def _count_wall_neighbours(self, map_obj: Map, x: int, y: int) -> int:
        """Count walls in the 8-cell Moore neighbourhood (out-of-bounds = wall)."""
        count = 0
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                nx, ny = x + dx, y + dy
                if not map_obj.in_bounds(nx, ny) or map_obj.grid[ny][nx] == 1:
                    count += 1
        return count
