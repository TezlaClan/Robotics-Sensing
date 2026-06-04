"""
room_generator.py

Generates maps composed of rooms connected by corridors.

Properties:
- Large open areas
- Variable room sizes
- Corridors between rooms
- More realistic navigation than mazes
"""

from typing import List, Tuple

from core.map import Map
from maps.base_generator import BaseMapGenerator


class RoomGenerator(BaseMapGenerator):
    def _generate_map(self) -> Map:
        map_obj = Map(self.width, self.height)

        # Start with all walls
        self._fill_map(map_obj, 1)

        rooms: List[Tuple[int, int, int, int]] = []

        room_count = self.rng.randint(4, 10)

        for _ in range(room_count):
            room = self._create_random_room(map_obj)

            if not self._room_overlaps(room, rooms):
                self._carve_room(map_obj, room)

                if rooms:
                    # Connect to previous room
                    prev_room = rooms[-1]
                    self._connect_rooms(map_obj, prev_room, room)

                rooms.append(room)

        return map_obj

    # =========================
    # Room Creation
    # =========================

    def _create_random_room(self, map_obj: Map) -> Tuple[int, int, int, int]:
        """
        Returns a room defined by (x1, y1, x2, y2)
        """

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

            # Check overlap with margin of 1 tile
            if (x1 <= ox2 + 1 and x2 >= ox1 - 1 and
                y1 <= oy2 + 1 and y2 >= oy1 - 1):
                return True

        return False

    def _carve_room(self, map_obj: Map, room: Tuple[int, int, int, int]):
        x1, y1, x2, y2 = room

        for y in range(y1, y2):
            for x in range(x1, x2):
                map_obj.set_cell(x, y, 0)

    # =========================
    # Corridor Connection
    # =========================

    def _connect_rooms(
        self,
        map_obj: Map,
        room1: Tuple[int, int, int, int],
        room2: Tuple[int, int, int, int]
    ):
        """
        Connect two rooms with a corridor (L-shaped)
        """

        x1, y1 = self._room_center(room1)
        x2, y2 = self._room_center(room2)

        if self.rng.random() < 0.5:
            # Horizontal then vertical
            self._carve_h_corridor(map_obj, x1, x2, y1)
            self._carve_v_corridor(map_obj, y1, y2, x2)
        else:
            # Vertical then horizontal
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