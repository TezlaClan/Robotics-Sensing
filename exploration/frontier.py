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
    def __init__(self, gain_weight: float = 0.0):
        self.visited_targets = set()
        self.target_count = 0
        # Frontier selection: 0 = pure nearest (greedy). >0 trades expected
        # information gain (unknown cells around the frontier) against travel
        # distance, so an agent prefers frontiers that reveal more per step rather
        # than nibbling the closest. EXPERIMENTAL - gated behind an A/B (avg steps
        # must fall with no completion/accuracy regression).
        self.gain_weight = gain_weight

    # =========================
    # Main API
    # =========================

    def choose_target(
        self,
        agent,
        internal_map,
        agents=None,
    ) -> Optional[GridPosition]:
        """
        Select an exploration frontier for `agent`.

        Multi-agent: frontiers are partitioned between agents by *proximity*, not
        by who picks first. A single multi-source BFS from every active agent
        labels each known-free cell with its nearest agent (a Voronoi partition),
        and this agent takes the nearest frontier in its own region. So a frontier
        always goes to the agent actually closest to it - no agent gets blocked
        from a near frontier by another's earlier claim (which previously forced
        long backtracks). If this agent owns no frontier, it falls back to its
        nearest reachable one so it always has something to do.

        Single-agent: just the nearest reachable frontier.
        """

        start = agent._to_grid(agent.believed_position)

        active = [a for a in (agents or []) if not getattr(a, "finished", False)]
        if len(active) > 1:
            target = self._voronoi_frontier(internal_map, agent, active)
        else:
            target = self._nearest_reachable_frontier(internal_map, start)

        if target is None:
            dprint(f"[Exploration] No reachable frontiers found (agent at {start})")
            return None

        dprint(f"[Exploration] Agent {agent.id} at {start} -> frontier {target}")
        return target

    def _voronoi_frontier(self, internal_map, agent, active) -> Optional[GridPosition]:
        """
        Multi-source BFS over known-free cells from every active agent's cell.
        Each cell is owned by its nearest agent (ties broken by lowest id, via
        seeding order). Returns the nearest frontier owned by `agent`, or - if it
        owns none - its nearest reachable frontier regardless of ownership.
        """
        height = len(internal_map)
        width = len(internal_map[0])
        directions = [(1, 0), (-1, 0), (0, 1), (0, -1)]

        owner = {}
        queue = deque()
        # Seed in ascending id order so equidistant cells go to the lower id.
        for a in sorted(active, key=lambda a: a.id):
            cell = a._to_grid(a.believed_position)
            cx, cy = cell
            if 0 <= cx < width and 0 <= cy < height and cell not in owner:
                owner[cell] = a.id
                queue.append(cell)

        my_start = agent._to_grid(agent.believed_position)
        owned = []   # (order, cell) for this agent's frontiers, BFS order = distance
        order = 0

        while queue:
            cx, cy = queue.popleft()
            cell = (cx, cy)

            if (owner[cell] == agent.id and cell != my_start
                    and self.is_frontier(internal_map, cell)):
                # Pure-nearest (gain_weight == 0): the first owned frontier in BFS
                # order IS the nearest, so return it immediately - identical to the
                # previous behaviour.
                if self.gain_weight <= 0:
                    return cell
                owned.append((order, cell))
                order += 1

            for dx, dy in directions:
                nx, ny = cx + dx, cy + dy
                if not (0 <= nx < width and 0 <= ny < height):
                    continue
                if (nx, ny) in owner:
                    continue
                # Expand only through known-free space so frontiers sit at the
                # edge of a corridor the agent can actually traverse.
                if self._is_free(internal_map[ny][nx]):
                    owner[(nx, ny)] = owner[cell]
                    queue.append((nx, ny))

        if not owned:
            # Own no frontier in our region: take the nearest reachable one anyway.
            return self._nearest_reachable_frontier(internal_map, my_start)

        # Gain-weighted: minimise (distance proxy) - gain_weight * (info gain).
        best_cell, best_score = None, None
        for o, cell in owned:
            score = o - self.gain_weight * self._info_gain(internal_map, cell)
            if best_score is None or score < best_score:
                best_score, best_cell = score, cell
        return best_cell

    def _info_gain(self, internal_map, cell) -> int:
        """Cheap expected-gain proxy: unknown cells in the 3x3 around `cell`."""
        x, y = cell
        h = len(internal_map)
        w = len(internal_map[0])
        g = 0
        for ny in range(max(0, y - 1), min(h, y + 2)):
            row = internal_map[ny]
            for nx in range(max(0, x - 1), min(w, x + 2)):
                if self._is_unknown(row[nx]):
                    g += 1
        return g

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

            if (cx, cy) != start and self.is_frontier(internal_map, (cx, cy)):
                return (cx, cy)

            for dx, dy in directions:
                nx, ny = cx + dx, cy + dy
                if not (0 <= nx < width and 0 <= ny < height):
                    continue
                if (nx, ny) in visited:
                    continue
                if self._is_free(internal_map[ny][nx]):
                    visited.add((nx, ny))
                    queue.append((nx, ny))

        return None

    def furthest_reachable_frontier(
        self,
        internal_map: List[List[float]],
        start: GridPosition,
        exclude=None,
    ) -> Optional[GridPosition]:
        """
        BFS outward from `start` through known-free cells, returning the FURTHEST
        frontier reachable (the last one reached in BFS order, which visits cells
        in non-decreasing path distance). Used by search/survey mode to sweep the
        agent across the known region's perimeter so its boundary walls get
        re-observed. `exclude` is a set of frontier cells to skip (already surveyed
        this episode) so search can move on instead of looping back. Returns None
        if no eligible frontier is reachable through free space.
        """
        height = len(internal_map)
        width = len(internal_map[0])
        exclude = exclude or set()

        sx, sy = start
        if not (0 <= sx < width and 0 <= sy < height):
            return None

        directions = [(1, 0), (-1, 0), (0, 1), (0, -1)]

        visited = {start}
        queue = deque([start])
        furthest = None

        while queue:
            cx, cy = queue.popleft()

            if ((cx, cy) != start and (cx, cy) not in exclude
                    and self.is_frontier(internal_map, (cx, cy))):
                furthest = (cx, cy)  # BFS order is by distance, so last seen = far

            for dx, dy in directions:
                nx, ny = cx + dx, cy + dy
                if not (0 <= nx < width and 0 <= ny < height):
                    continue
                if (nx, ny) in visited:
                    continue
                if self._is_free(internal_map[ny][nx]):
                    visited.add((nx, ny))
                    queue.append((nx, ny))

        return furthest

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