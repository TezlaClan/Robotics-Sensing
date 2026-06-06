"""
obstacle_generator.py

Generates a mostly-open arena scattered with wall obstacles / pillars.

Properties:
- Large open space (good for testing open-area navigation)
- Randomly placed single-cell and small clustered obstacles
- Solid border wall, single connected free area
"""

from core.map import Map
from maps.base_generator import BaseMapGenerator


class ObstacleGenerator(BaseMapGenerator):
    OBSTACLE_DENSITY = 0.20   # fraction of cells turned into obstacles

    def _generate_map(self) -> Map:
        map_obj = Map(self.width, self.height)

        # =========================
        # 1. Scatter obstacles across the whole map
        # =========================
        self._random_wall_density(map_obj, self.OBSTACLE_DENSITY)

        # =========================
        # 2. Keep a solid border wall
        # =========================
        for x in range(self.width):
            map_obj.set_cell(x, 0, 1)
            map_obj.set_cell(x, self.height - 1, 1)
        for y in range(self.height):
            map_obj.set_cell(0, y, 1)
            map_obj.set_cell(self.width - 1, y, 1)

        # =========================
        # 3. Ensure one connected open area
        # (fill any free pockets sealed off by obstacle clusters)
        # =========================
        self._keep_largest_region(map_obj)

        return map_obj
