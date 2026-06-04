"""
frontier.py

Implements Frontier-Based Exploration.

Agents move toward the boundary between:
- known space
- unknown space

This is a standard exploration algorithm in robotics.
"""

from typing import List, Tuple, Optional
import math

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
        Select a frontier cell as the next exploration target.
        """

        frontiers = self._find_frontiers(internal_map)

        if not frontiers:
            print(f"[Exploration] No frontiers found!")
            return None

        agent_pos = agent._to_grid(agent.believed_position)
        print(f"[Exploration] Found {len(frontiers)} frontiers, agent at {agent_pos}")

        # Choose nearest frontier
        best = min(
            frontiers,
            key=lambda f: self._distance(agent_pos, f)
        )

        # Track targets to detect cycling
        self.target_count += 1
        if best in self.visited_targets:
            print(f"[Exploration] WARNING: Revisiting target {best} (count: {self.target_count})")
        else:
            self.visited_targets.add(best)
            print(f"[Exploration] New target: {best} (total targets: {len(self.visited_targets)})")

        return best

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

        print(f"[Exploration] Frontier detection: {free_cells} free cells, {cells_with_unknown} are frontiers")
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