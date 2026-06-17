"""
astar.py

Implements A* pathfinding on a probabilistic occupancy grid.

Supports:
- Unknown space traversal (higher cost)
- Avoids walls
- Works with agent internal maps
"""

import heapq
import math
from typing import Tuple, List, Dict, Optional

from utils.debug import dprint

GridPosition = Tuple[int, int]


class AStarPlanner:
    def __init__(self, agent_radius=0.3, nav_locked_only=False):
      self.agent_radius = agent_radius
      # When True, only LOCKED (confirmed) believed-walls block navigation; an
      # unlocked >wall cell (still accumulating evidence, e.g. a transient phantom
      # from a drift episode) is passable-but-penalised. Collision handling on the
      # TRUE map still stops real walls, so this is safe, and a wall must be
      # confirmed (locked) before it can seal a route - phantoms that never lock
      # never seal. Requires the `locked` grid to be passed to plan().
      self.nav_locked_only = nav_locked_only

    # =========================
    # Main API
    # =========================

    def plan(
        self,
        start: GridPosition,
        goal: GridPosition,
        internal_map: List[List[float]],
        allow_walls: bool = False,
        locked=None,
    ) -> List[GridPosition]:
        """
        Compute path from start to goal.

        allow_walls: recovery mode. When True, believed-walls are traversable at
        a high cost, so the agent can push through a *suspected* wall (e.g. a
        noisy false-positive that sealed a corridor). Real walls still block in
        movement, so this is safe; phantom walls get re-observed and corrected.

        locked: the agent's lock grid. With nav_locked_only set, only locked walls
        block; unlocked >wall cells are traversable at a penalty.
        """

        height = len(internal_map)
        width = len(internal_map[0])

        def in_bounds(cell):
            return 0 <= cell[0] < width and 0 <= cell[1] < height

        # Can't plan from / to a cell outside the map (e.g. if a bad pose
        # estimate puts the start off-grid).
        if not in_bounds(start) or not in_bounds(goal):
            dprint(f"[A*] Start {start} or goal {goal} out of bounds")
            return []

        if start == goal:
            dprint(f"[A*] Start == Goal: {start}")
            return []

        dprint(f"[A*] Planning from {start} to {goal}")

        open_set = []
        heapq.heappush(open_set, (0, start))

        came_from: Dict[GridPosition, Optional[GridPosition]] = {}
        g_score: Dict[GridPosition, float] = {start: 0}

        f_score: Dict[GridPosition, float] = {
            start: self._heuristic(start, goal)
        }

        iterations = 0
        while open_set:
            iterations += 1
            _, current = heapq.heappop(open_set)

            if current == goal:
                path = self._reconstruct_path(came_from, current)
                dprint(f"[A*] Path found in {iterations} iterations: {len(path)} cells")
                return path

            for neighbour in self._get_neighbours(current, internal_map, allow_walls, locked):
                tentative_g = g_score[current] + self._cost(neighbour, internal_map, allow_walls, locked)

                if neighbour not in g_score or tentative_g < g_score[neighbour]:
                  came_from[neighbour] = current
                  g_score[neighbour] = tentative_g

                  f = tentative_g + self._heuristic(neighbour, goal)

                  heapq.heappush(open_set, (f, neighbour))

        # No path found
        dprint(f"[A*] No path found after {iterations} iterations")
        return []

    # =========================
    # Neighbour Logic
    # =========================

    def _is_blocked(self, x, y, internal_map, locked=None):
      """
      Check if a cell blocks navigation. A >wall cell normally blocks; with
      nav_locked_only it blocks only once LOCKED (confirmed) - an unlocked >wall
      cell is treated as passable here (the cost function penalises it).
      """
      if internal_map[y][x] > 0.6:
          if self.nav_locked_only and locked is not None and not locked[y][x]:
              return False  # unconfirmed wall: don't block, route may use it
          return True

      return False

    def _get_neighbours(
        self,
        node: GridPosition,
        internal_map: List[List[float]],
        allow_walls: bool = False,
        locked=None,
    ) -> List:
      x, y = node
      width = len(internal_map[0])
      height = len(internal_map)

      directions = [
          (1, 0), (-1, 0),
          (0, 1), (0, -1),
      ]

      neighbours = []

      for dx, dy in directions:
          nx, ny = x + dx, y + dy

          if 0 <= nx < width and 0 <= ny < height:
              if allow_walls or not self._is_blocked(nx, ny, internal_map, locked):
                  neighbours.append((nx, ny))

      if not neighbours:
          dprint(f"[A*] WARNING: No neighbours for node {node}!")
          dprint(f"[A*] Map prob at node: {internal_map[y][x]}")

      return neighbours

    # =========================
    # Cost Function
    # =========================

    def _cost(self, node: GridPosition, internal_map, allow_walls: bool = False,
              locked=None) -> float:
        """
        Uniform cost - all free cells cost the same.
        This encourages shortest path, not wall-hugging.
        """
        x, y = node
        prob = internal_map[y][x]

        # Believed wall.
        if prob > 0.6:
            # Unconfirmed (unlocked) wall under nav_locked_only: passable but
            # penalised, so a confirmed-free route is preferred when one exists,
            # yet the agent will route through a suspected phantom rather than
            # treat it as a seal.
            if self.nav_locked_only and locked is not None and not locked[y][x]:
                return 5.0
            # Confirmed wall: impassable normally, very expensive (not infinite)
            # in recovery mode so a free route is always preferred when one exists.
            return 1000.0 if allow_walls else float("inf")

        # All free/unknown cells have equal cost
        return 1.0


    # =========================
    # Heuristic
    # =========================

    def _heuristic(self, a: GridPosition, b: GridPosition) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])

    # =========================
    # Path Reconstruction
    # =========================

    def _reconstruct_path(
        self,
        came_from: Dict[GridPosition, GridPosition],
        current: GridPosition,
    ) -> List[GridPosition]:

        path = [current]

        while current in came_from:
            current = came_from[current]
            path.append(current)

        path.reverse()
        
        # Remove start node from path - agent should only follow waypoints, not current position
        if path:
            path.pop(0)

        return path