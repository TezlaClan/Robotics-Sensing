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
    def __init__(self, agent_radius=0.3):
      self.agent_radius = agent_radius

    # =========================
    # Main API
    # =========================

    def plan(
        self,
        start: GridPosition,
        goal: GridPosition,
        internal_map: List[List[float]],
    ) -> List[GridPosition]:
        """
        Compute path from start to goal.
        """

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

            for neighbour in self._get_neighbours(current, internal_map):
                tentative_g = g_score[current] + self._cost(neighbour, internal_map)

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

    def _is_blocked(self, x, y, internal_map, inflation_radius):
      """
      Check if a cell is blocked (wall or unknown).
      No inflation radius - just check the cell itself.
      """
      height = len(internal_map)
      width = len(internal_map[0])

      # Hard wall
      if internal_map[y][x] > 0.6:
          return True

      return False

    def _get_neighbours(
        self,
        node: GridPosition,
        internal_map: List[List[float]],
    ) -> List:
      x, y = node
      width = len(internal_map[0])
      height = len(internal_map)
      inflation_radius = self.agent_radius

      directions = [
          (1, 0), (-1, 0),
          (0, 1), (0, -1),
      ]

      neighbours = []

      for dx, dy in directions:
          nx, ny = x + dx, y + dy

          if 0 <= nx < width and 0 <= ny < height:
              if not self._is_blocked(nx, ny, internal_map, inflation_radius):
                  neighbours.append((nx, ny))

      if not neighbours:
          dprint(f"[A*] WARNING: No neighbours for node {node}!")
          dprint(f"[A*] Map prob at node: {internal_map[y][x]}")
          for dx, dy in directions:
              nx, ny = x + dx, y + dy
              if 0 <= nx < width and 0 <= ny < height:
                  blocked = self._is_blocked(nx, ny, internal_map, inflation_radius)
                  prob = internal_map[ny][nx]
                  dprint(f"[A*]   ({nx}, {ny}): prob={prob:.2f}, blocked={blocked}")
      
      return neighbours

    # =========================
    # Cost Function
    # =========================

    def _cost(self, node: GridPosition, internal_map) -> float:
        """
        Uniform cost - all free cells cost the same.
        This encourages shortest path, not wall-hugging.
        """
        x, y = node
        prob = internal_map[y][x]

        # Hard block
        if prob > 0.6:
            return float("inf")

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