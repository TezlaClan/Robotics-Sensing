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
from collections import deque
from typing import Tuple, List, Dict, Optional

from utils.debug import dprint

GridPosition = Tuple[int, int]


def true_path_distance(map_obj, start_cell, goal_cell):
    """
    A* over the TRUE map (full knowledge): the shortest 4-connected free-space
    path length, in cells, from start_cell to goal_cell. Returns float("inf") if
    no path exists.

    This is an ORACLE distance for measurement only - it sees the whole map, so it
    reports exactly how far the goal is along the geometry the agent must navigate
    (4-connected, matching the planner), not the straight-line distance. In a maze
    the two differ enormously: the goal can be a few cells away in a straight line
    yet hundreds of cells away along the only free path. Not used by the agents.
    """
    sx, sy = start_cell
    gx, gy = goal_cell
    w, h = map_obj.width, map_obj.height
    grid = map_obj.grid

    if not (0 <= sx < w and 0 <= sy < h and 0 <= gx < w and 0 <= gy < h):
        return float("inf")
    if grid[sy][sx] == 1 or grid[gy][gx] == 1:
        return float("inf")
    if (sx, sy) == (gx, gy):
        return 0.0

    def heur(x, y):
        return abs(x - gx) + abs(y - gy)  # Manhattan: admissible on a 4-grid

    open_set = [(heur(sx, sy), 0, (sx, sy))]
    g_score = {(sx, sy): 0}

    while open_set:
        _, g, (cx, cy) = heapq.heappop(open_set)
        if (cx, cy) == (gx, gy):
            return float(g)
        if g > g_score.get((cx, cy), float("inf")):
            continue  # stale heap entry
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nx, ny = cx + dx, cy + dy
            if 0 <= nx < w and 0 <= ny < h and grid[ny][nx] == 0:
                ng = g + 1
                if ng < g_score.get((nx, ny), float("inf")):
                    g_score[(nx, ny)] = ng
                    heapq.heappush(open_set, (ng + heur(nx, ny), ng, (nx, ny)))

    return float("inf")


class AStarPlanner:
    def __init__(self, agent_radius=0.3, nav_locked_only=False,
                 wall_affinity=False, wall_affinity_weight=1.0,
                 wall_affinity_comfort=None, sensor_range=8.0):
      self.agent_radius = agent_radius
      # When True, only LOCKED (confirmed) believed-walls block navigation; an
      # unlocked >wall cell (still accumulating evidence, e.g. a transient phantom
      # from a drift episode) is passable-but-penalised. Collision handling on the
      # TRUE map still stops real walls, so this is safe, and a wall must be
      # confirmed (locked) before it can seal a route - phantoms that never lock
      # never seal. Requires the `locked` grid to be passed to plan().
      self.nav_locked_only = nav_locked_only

      # Wall-affinity (perception-aware planning). In open maps (caves) the SLAM
      # filter has no geometry to localize against when no walls are in sensor
      # range, so the believed pose drifts. When on, routing through a cell whose
      # nearest believed wall is FURTHER than `wall_affinity_comfort` cells is
      # penalised (linearly in the excess distance), biasing paths to keep walls
      # within sensing range so the filter stays constrained. It is a SOFT penalty,
      # not a hard constraint: when no near-wall route to the target exists (the
      # goal is out in the open), A* still returns the least-penalty path - so the
      # constraint effectively lifts itself rather than failing to plan. Off in
      # recovery (allow_walls) mode. Default off until an A/B supports it.
      self.wall_affinity = wall_affinity
      self.wall_affinity_weight = wall_affinity_weight
      # Cells with a wall within this many cells incur no penalty; default to the
      # sensor range (a wall must merely be sensible).
      self.wall_affinity_comfort = (
          sensor_range if wall_affinity_comfort is None else wall_affinity_comfort
      )

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

        # Perception-aware wall-distance field (only when affinity is active and
        # we are not in wall-crossing recovery). Computed once per plan, O(W*H).
        wall_dist = None
        if self.wall_affinity and not allow_walls:
            wall_dist = self._wall_distance_field(internal_map)

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
                tentative_g = g_score[current] + self._cost(neighbour, internal_map, allow_walls, locked, wall_dist)

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
              locked=None, wall_dist=None) -> float:
        """
        Uniform cost - all free cells cost the same, EXCEPT when wall-affinity is
        active: a free cell whose nearest believed wall is beyond the comfort
        radius is penalised in proportion to the excess distance, so routes prefer
        to keep walls within sensing range (better localization in open areas).
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

        # Free/unknown cell: base cost 1, plus the wall-affinity penalty (if any).
        if wall_dist is not None:
            return 1.0 + self.wall_affinity_weight * wall_dist[y][x]
        return 1.0

    def _wall_distance_field(self, internal_map) -> List[List[float]]:
        """
        Per-cell wall-affinity PENALTY field (0 = pose well-constrained here).

        What matters for localization is not nearness to *a* wall but having walls
        across DIFFERENT axes within sensing range. A lone nearby wall (or hugging
        one flat wall) leaves motion *along* that wall unconstrained - the corridor
        aperture problem - and in practice makes drift worse, not better. So this
        scores each free cell by per-axis wall coverage: for the x-axis, the nearer
        wall scanning left/right; for the y-axis, the nearer wall scanning up/down
        (both capped at the sensor range). An axis with no wall within `comfort`
        contributes a penalty equal to its excess distance; a cell with a wall in
        range on BOTH axes (a corner / junction / narrow spot, where pose is fully
        pinned) gets 0. The open middle of a long corridor - where along-corridor
        drift happens - is penalised on its open axis, biasing routes toward the
        structure that actually constrains the filter.
        """
        height = len(internal_map)
        width = len(internal_map[0])
        comfort = self.wall_affinity_comfort
        rng = int(comfort) + 1  # how far to scan for a wall before giving up
        pen = [[0.0] * width for _ in range(height)]

        def axis_dist(x, y, dx, dy):
            """Cells to the nearest wall in +/- (dx,dy) from (x,y); rng+1 if none."""
            best = rng + 1
            for sx, sy in ((dx, dy), (-dx, -dy)):
                cx, cy = x + sx, y + sy
                d = 1
                while 0 <= cx < width and 0 <= cy < height and d <= rng:
                    if internal_map[cy][cx] > 0.6:
                        if d < best:
                            best = d
                        break
                    cx += sx
                    cy += sy
                    d += 1
            return best

        for y in range(height):
            row = internal_map[y]
            for x in range(width):
                if row[x] > 0.6:
                    continue  # walls don't get a free-cell penalty
                xd = axis_dist(x, y, 1, 0)
                yd = axis_dist(x, y, 0, 1)
                p = 0.0
                if xd > comfort:
                    p += xd - comfort
                if yd > comfort:
                    p += yd - comfort
                if p:
                    pen[y][x] = p
        return pen


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