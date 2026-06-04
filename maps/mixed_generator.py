"""
mixed_generator.py

Generates hybrid maps combining:
- Maze regions
- Open rooms
- Corridors
- Random variation

Designed to provide diverse navigation challenges.
"""

from typing import Tuple, List

from core.map import Map
from maps.base_generator import BaseMapGenerator
from maps.maze_generator import MazeGenerator


class MixedGenerator(BaseMapGenerator):
    def _generate_map(self) -> Map:
        map_obj = Map(self.width, self.height)

        # Start with all walls
        self._fill_map(map_obj, 1)

        # =========================
        # 1. Create Maze Subsection
        # =========================
        maze_width = self.width // 2
        maze_height = self.height // 2

        maze_gen = MazeGenerator(maze_width, maze_height, self.rng_manager)
        maze_map = maze_gen._generate_map()  # NOTE: internal call

        # Place maze into main map
        offset_x = self.rng.randint(0, self.width - maze_width)
        offset_y = self.rng.randint(0, self.height - maze_height)

        for y in range(maze_map.height):
            for x in range(maze_map.width):
                if maze_map.grid[y][x] == 0:
                    map_obj.set_cell(x + offset_x, y + offset_y, 0)

        # =========================
        # 2. Add Random Rooms
        # =========================
        rooms: List[Tuple[int, int, int, int]] = []

        room_count = self.rng.randint(3, 8)

        for _ in range(room_count):
            room = self._create_random_room(map_obj)

            if not self._room_overlaps(room, rooms):
                self._carve_room(map_obj, room)

                if rooms:
                    prev_room = rooms[-1]
                    self._connect_rooms(map_obj, prev_room, room)

                rooms.append(room)

        # =========================
        # 3. Add Random Corridors
        # =========================
        corridor_count = self.rng.randint(3, 6)

        for _ in range(corridor_count):
            self._random_corridor(map_obj)

        return map_obj

    # =========================
    # Room Logic (same as room generator)
    # =========================

    def _create_random_room(self, map_obj: Map) -> Tuple[int, int, int, int]:
        room_width = self.rng.randint(4, 10)
        room_height = self.rng.randint(4, 10)

        x = self.rng.randint(1, map_obj.width - room_width - 1)
        y = self.rng.randint(1, map_obj.height - room_height - 1)

        return (x, y, x + room_width, y + room_height)

    def _room_overlaps(
        self, room: Tuple[int, int, int, int], rooms: List[Tuple[int, int, int, int]]
    ) -> bool:
        x1, y1, x2, y2 = room

        for other in rooms:
            ox1, oy1, ox2, oy2 = other

            if (x1 <= ox2 + 1 and x2 >= ox1 - 1 and
                y1 <= oy2 + 1 and y2 >= oy1 - 1):
                return True

        return False

    def _carve_room(self, map_obj: Map, room: Tuple[int, int, int, int]):
        x1, y1, x2, y2 = room

        for y in range(y1, y2):
            for x in range(x1, x2):
                map_obj.set_cell(x, y, 0)

    def _connect_rooms(
        self,
        map_obj: Map,
        room1: Tuple[int, int, int, int],
        room2: Tuple[int, int, int, int]
    ):
        x1, y1 = self._room_center(room1)
        x2, y2 = self._room_center(room2)

        if self.rng.random() < 0.5:
            self._carve_h_corridor(map_obj, x1, x2, y1)
            self._carve_v_corridor(map_obj, y1, y2, x2)
        else:
            self._carve_v_corridor(map_obj, y1, y2, x1)
            self._carve_h_corridor(map_obj, x1, x2, y2)

    def _room_center(self, room: Tuple[int, int, int, int]) -> Tuple[int, int]:
        x1, y1, x2, y2 = room
        return ((x1 + x2) // 2, (y1 + y2) // 2)

    def _carve_h_corridor(self, map_obj: Map, x1: int, x2: int, y: int):
        for x in range(min(x1, x2), max(x1, x2) + 1):
            map_obj.set_cell(x, y, 0)

    def _carve_v_corridor(self, map_obj: Map, y1: int, y2: int, x: int):
        for y in range(min(y1, y2), max(y1, y2) + 1):
            map_obj.set_cell(x, y, 0)

    # =========================
    # Corridor Logic
    # =========================

    def _random_corridor(self, map_obj: Map):
        """
        Carves a random straight corridor across the map.
        """
        horizontal = self.rng.random() < 0.5

        if horizontal:
            y = self.rng.randint(1, map_obj.height - 2)
            for x in range(1, map_obj.width - 1):
                map_obj.set_cell(x, y, 0)
        else:
            x = self.rng.randint(1, map_obj.width - 2)
            for y in range(1, map_obj.height - 1):
                map_obj.set_cell(x, y, 0)