"""
bsp_generator.py

Generates building-like layouts via Binary Space Partitioning (BSP).

The interior is recursively split into sub-regions; each leaf region gets a
rectangular room, and sibling regions are joined by corridors. Connecting at
every split guarantees all rooms form a single connected tree (so the map is
always solvable).

Properties:
- Grid of rectangular rooms of varying size
- Connected by short corridors / doorways
- Tighter, more structured than the room generator
"""

from typing import Tuple

from core.map import Map
from maps.base_generator import BaseMapGenerator

Point = Tuple[int, int]


class BSPGenerator(BaseMapGenerator):
    MIN_LEAF = 8      # regions larger than this may be split further
    MIN_CHILD = 4     # each side of a split must be at least this wide/tall
    MIN_ROOM = 3      # smallest room dimension

    def _generate_map(self) -> Map:
        map_obj = Map(self.width, self.height)

        # Everything starts as wall; rooms/corridors are carved out.
        self._fill_map(map_obj, 1)

        # Partition the interior (inset by 1 so the border stays walled).
        self._partition(map_obj, 1, 1, self.width - 2, self.height - 2)

        return map_obj

    # =========================
    # Recursive partitioning
    # =========================

    def _partition(self, map_obj: Map, x: int, y: int, w: int, h: int) -> Point:
        """
        Split region (x, y, w, h) recursively. Returns a representative point
        (a room centre) within the region so the parent can connect siblings.
        """

        # Decide whether (and how) to split.
        can_split_w = w > self.MIN_LEAF
        can_split_h = h > self.MIN_LEAF

        if can_split_w and can_split_h:
            split_horizontal = self.rng.random() < 0.5
        elif can_split_w:
            split_horizontal = False  # split along the x axis
        elif can_split_h:
            split_horizontal = True   # split along the y axis
        else:
            # Leaf region: carve a room.
            return self._carve_leaf_room(map_obj, x, y, w, h)

        if split_horizontal:
            split = self.rng.randint(self.MIN_CHILD, h - self.MIN_CHILD)
            c1 = self._partition(map_obj, x, y, w, split)
            c2 = self._partition(map_obj, x, y + split, w, h - split)
        else:
            split = self.rng.randint(self.MIN_CHILD, w - self.MIN_CHILD)
            c1 = self._partition(map_obj, x, y, split, h)
            c2 = self._partition(map_obj, x + split, y, w - split, h)

        # Join the two halves so the whole tree stays connected.
        self._connect(map_obj, c1, c2)
        return c1

    # =========================
    # Rooms & corridors
    # =========================

    def _carve_leaf_room(self, map_obj: Map, x: int, y: int, w: int, h: int) -> Point:
        """Carve a room inside the region, leaving a margin, and return its centre."""
        rw = self.rng.randint(min(self.MIN_ROOM, w), max(self.MIN_ROOM, w - 1))
        rh = self.rng.randint(min(self.MIN_ROOM, h), max(self.MIN_ROOM, h - 1))

        rw = min(rw, w)
        rh = min(rh, h)

        rx = x + self.rng.randint(0, w - rw)
        ry = y + self.rng.randint(0, h - rh)

        # _carve_rectangle is exclusive of x2/y2, so pass rx+rw / ry+rh.
        self._carve_rectangle(map_obj, rx, ry, rx + rw, ry + rh)

        return (rx + rw // 2, ry + rh // 2)

    def _connect(self, map_obj: Map, a: Point, b: Point):
        """L-shaped corridor between two points."""
        ax, ay = a
        bx, by = b

        if self.rng.random() < 0.5:
            self._carve_h_corridor(map_obj, ax, bx, ay)
            self._carve_v_corridor(map_obj, ay, by, bx)
        else:
            self._carve_v_corridor(map_obj, ay, by, ax)
            self._carve_h_corridor(map_obj, ax, bx, by)

    def _carve_h_corridor(self, map_obj: Map, x1: int, x2: int, y: int):
        for x in range(min(x1, x2), max(x1, x2) + 1):
            map_obj.set_cell(x, y, 0)

    def _carve_v_corridor(self, map_obj: Map, y1: int, y2: int, x: int):
        for y in range(min(y1, y2), max(y1, y2) + 1):
            map_obj.set_cell(x, y, 0)
