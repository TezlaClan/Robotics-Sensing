"""
frontier.py

Implements Frontier-Based Exploration.

Agents move toward the boundary between:
- known space
- unknown space

This is a standard exploration algorithm in robotics.
"""

from typing import List, Tuple, Optional
from collections import deque
import math

from utils.debug import dprint

GridPosition = Tuple[int, int]


class FrontierExploration:
    def __init__(self):
        self.visited_targets = set()
        self.target_count = 0

    # =========================
    # Main API
    # =========================

    def choose_target(
        self,
        agent,
        internal_map
    ) -> Optional[GridPosition]:
        """
        Select the nearest *reachable* frontier as the next exploration target.

        Reachability is measured by a BFS over known-free cells from the agent's
        position, so the chosen frontier is the closest one along a corridor the
        agent can actually traverse - not just the closest as the crow flies.
        This avoids picking a frontier that is near in straight-line distance but
        only reachable by going all the way around the maze.
        """

        start = agent._to_grid(agent.believed_position)
        target = self._nearest_reachable_frontier(internal_map, start)

        if target is None:
            dprint(f"[Exploration] No reachable frontiers found (agent at {start})")
            return None

        dprint(f"[Exploration] Agent at {start}, nearest reachable frontier: {target}")
        return target

    def _nearest_reachable_frontier(
        self,
        internal_map: List[List[float]],
        start: GridPosition,
    ) -> Optional[GridPosition]:
        """
        BFS outward from `start` through known-free cells, returning the first
        frontier cell encountered (i.e. nearest by reachable path distance).
        """

        height = len(internal_map)
        width = len(internal_map[0])

        sx, sy = start
        if not (0 <= sx < width and 0 <= sy < height):
            return None

        directions = [(1, 0), (-1, 0), (0, 1), (0, -1)]

        visited = {start}
        queue = deque([start])

        while queue:
            cx, cy = queue.popleft()

            # The first frontier we reach is the nearest reachable one.
            # (Skip the agent's own cell so we always pick something to move
            # toward rather than a zero-length target.)
            if (cx, cy) != start and self.is_frontier(internal_map, (cx, cy)):
                return (cx, cy)

            for dx, dy in directions:
                nx, ny = cx + dx, cy + dy

                if not (0 <= nx < width and 0 <= ny < height):
                    continue
                if (nx, ny) in visited:
                    continue

                # Only expand through cells we know are free, so every reached
                # frontier sits at the edge of a corridor the agent can follow.
                if self._is_free(internal_map[ny][nx]):
                    visited.add((nx, ny))
                    queue.append((nx, ny))

        return None

    def nearest_unknown(
        self,
        internal_map: List[List[float]],
        start: GridPosition,
    ) -> Optional[GridPosition]:
        """
        Nearest unexplored cell, searching through ALL cells (walls included).

        Used for recovery when no frontier is reachable through known-free space
        - typically because a noisy false-positive reading sealed off the known
        region with a phantom wall. The agent heads here through believed-walls;
        real walls block in movement and phantom ones get corrected on the way.
        """
        height = len(internal_map)
        width = len(internal_map[0])

        sx, sy = start
        if not (0 <= sx < width and 0 <= sy < height):
            return None

        visited = {start}
        queue = deque([start])

        while queue:
            cx, cy = queue.popleft()

            if (cx, cy) != start and self._is_unknown(internal_map[cy][cx]):
                return (cx, cy)

            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nx, ny = cx + dx, cy + dy
                if 0 <= nx < width and 0 <= ny < height and (nx, ny) not in visited:
                    visited.add((nx, ny))
                    queue.append((nx, ny))

        return None

    def is_frontier(self, internal_map, cell) -> bool:
        """
        Check whether a single cell is still a frontier:
        a free cell that borders unknown space.

        Used by the agent to decide if it should keep heading to its
        current target or pick a new one.
        """
        x, y = cell

        height = len(internal_map)
        width = len(internal_map[0])

        if not (0 <= x < width and 0 <= y < height):
            return False

        if not self._is_free(internal_map[y][x]):
            return False

        return self._has_unknown_neighbour(internal_map, x, y)

    # =========================
    # Frontier Detection
    # =========================

    def _find_frontiers(
        self,
        internal_map: List[List[float]]
    ) -> List[GridPosition]:
        """
        Find all frontier cells.
        """

        height = len(internal_map)
        width = len(internal_map[0])

        frontiers = []
        free_cells = 0
        cells_with_unknown = 0

        for y in range(height):
            for x in range(width):

                if not self._is_free(internal_map[y][x]):
                    continue
                
                free_cells += 1

                if self._has_unknown_neighbour(internal_map, x, y):
                    frontiers.append((x, y))
                    cells_with_unknown += 1

        dprint(f"[Exploration] Frontier detection: {free_cells} free cells, {cells_with_unknown} are frontiers")
        return frontiers

    def _is_free(self, prob: float) -> bool:
        """
        Determine if a cell is considered free.
        """
        return prob < 0.4  # threshold

    def _is_unknown(self, prob: float) -> bool:
        """
        Determine if a cell is unknown.
        """
        return 0.4 <= prob <= 0.6

    def _has_unknown_neighbour(
        self,
        internal_map,
        x: int,
        y: int
    ) -> bool:
        """
        Check if this cell borders unknown space.
        """

        directions = [
            (1, 0), (-1, 0),
            (0, 1), (0, -1),
        ]

        for dx, dy in directions:
            nx, ny = x + dx, y + dy

            if 0 <= nx < len(internal_map[0]) and 0 <= ny < len(internal_map):
                if self._is_unknown(internal_map[ny][nx]):
                    return True

        return False

    # =========================
    # Utility
    # =========================

    def _distance(self, a: GridPosition, b: GridPosition) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])