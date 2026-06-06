"""
slam.py

Scan-matching localization (simplified SLAM).

Each update:
  1. Predict the new pose from odometry (motion + accumulated drift).
  2. Correct it by scan matching: for candidate cells near the prediction,
     compute what the agent *would* see from there (line-of-sight over its own
     internal map) and pick the cell whose predicted view best matches what was
     actually observed. Fuse prediction and match with a complementary filter.
  3. Enforce a hard constraint: the estimate can never lie inside a known wall -
     if the agent thinks it is in a wall, it must be wrong, so snap to the
     nearest free cell.

Why this beats odometry/the old heuristic:
  - Sensor noise only flips the occupied/free flag of an observed cell; it does
    NOT change *which* cells are observed, so the matched coordinate set is
    noise-free.
  - The predicted view depends on the candidate's position relative to walls, so
    near corners and corridors the match is sharp and pulls out drift. In open,
    featureless space the match is ambiguous and it leans on odometry - exactly
    how real scan matching degrades.
  - Walls are physically impossible positions and are rejected outright.
"""

from typing import Tuple, List, Optional
import math

from sensing.fov import visible_cells

Position = Tuple[float, float]
Observation = Tuple[int, int, bool]

# Internal-map occupancy threshold above which a cell is treated as a wall.
_WALL_THRESHOLD = 0.6


class SLAMLocalization:
    def __init__(
        self,
        noise_sigma: float = 0.1,
        sensor_range: float = 5,
        search_radius: int = 3,
        gain: float = 0.5,
        rng_manager=None,
    ):
        """
        noise_sigma:    per-step odometry drift std-dev (shared with odometry).
        sensor_range:   range used to predict the view from candidate poses.
        search_radius:  cells searched around the prediction (max correctable drift/step).
        gain:           0..1 pull toward the scan match (the wall constraint is
                        always enforced regardless of gain).
        """
        self.noise_sigma = noise_sigma
        self.sensor_range = int(sensor_range)
        self.search_radius = search_radius
        self.gain = gain

        self.rng_manager = rng_manager
        self.rng = rng_manager.behaviour_rng() if rng_manager else None

    # =========================
    # Main API
    # =========================

    def update(
        self,
        believed_position: Position,
        motion: Position,
        observations: List[Observation],
        internal_map: Optional[List[List[float]]] = None,
    ) -> Position:
        # -------------------------
        # 1. Odometry prediction (with drift)
        # -------------------------
        pred_x = believed_position[0] + motion[0]
        pred_y = believed_position[1] + motion[1]

        if self.rng is not None and self.noise_sigma > 0:
            pred_x += self.rng.gauss(0, self.noise_sigma)
            pred_y += self.rng.gauss(0, self.noise_sigma)

        est_x, est_y = pred_x, pred_y

        # -------------------------
        # 2. Scan-match correction (needs the map)
        # -------------------------
        if internal_map is not None:
            fix = self._scan_fix(observations, internal_map, pred_x, pred_y)
            if fix is not None:
                est_x = pred_x + self.gain * (fix[0] - pred_x)
                est_y = pred_y + self.gain * (fix[1] - pred_y)

            # -------------------------
            # 3. Hard constraint: never inside a known wall
            # -------------------------
            est_x, est_y = self._project_out_of_walls(est_x, est_y, internal_map)

        return (est_x, est_y)

    # =========================
    # Scan matching (FOV correlation)
    # =========================

    def _scan_fix(
        self,
        observations: List[Observation],
        internal_map: List[List[float]],
        pred_x: float,
        pred_y: float,
    ) -> Optional[Position]:
        """
        Pick the candidate cell (near the prediction) whose predicted field of
        view best matches the observed cells. Returns its centre, or None.
        """
        observed = {(ox, oy) for ox, oy, _ in observations}
        if not observed:
            return None

        height = len(internal_map)
        width = len(internal_map[0])

        def in_bounds(x, y):
            return 0 <= x < width and 0 <= y < height

        def is_blocked(x, y):
            return (not in_bounds(x, y)) or internal_map[y][x] > _WALL_THRESHOLD

        pcx, pcy = int(pred_x), int(pred_y)
        s = self.search_radius

        best_cell = None
        best_score = -1.0
        best_dist = 0.0

        for cy in range(pcy - s, pcy + s + 1):
            for cx in range(pcx - s, pcx + s + 1):
                # The agent stands on a cell it sees, and never inside a wall.
                if (cx, cy) not in observed or is_blocked(cx, cy):
                    continue

                expected = visible_cells(is_blocked, in_bounds, cx, cy, self.sensor_range)

                # Jaccard overlap between predicted and observed views.
                inter = len(expected & observed)
                union = len(expected | observed)
                score = inter / union if union else 0.0

                dist = math.hypot(cx + 0.5 - pred_x, cy + 0.5 - pred_y)

                # Best match wins; ties break toward the odometry prediction.
                if score > best_score or (score == best_score and dist < best_dist):
                    best_score = score
                    best_dist = dist
                    best_cell = (cx, cy)

        if best_cell is None:
            return None

        return (best_cell[0] + 0.5, best_cell[1] + 0.5)

    # =========================
    # Free-space constraint
    # =========================

    def _project_out_of_walls(self, x, y, internal_map) -> Position:
        """If (x, y) lies in a known wall, snap to the nearest free cell centre."""
        height = len(internal_map)
        width = len(internal_map[0])

        cx, cy = int(x), int(y)

        def is_wall(ix, iy):
            return (not (0 <= ix < width and 0 <= iy < height)
                    or internal_map[iy][ix] > _WALL_THRESHOLD)

        if not is_wall(cx, cy):
            return (x, y)

        # Expanding ring search for the closest non-wall cell.
        for radius in range(1, self.search_radius + 2):
            best = None
            best_d = float("inf")
            for dy in range(-radius, radius + 1):
                for dx in range(-radius, radius + 1):
                    if max(abs(dx), abs(dy)) != radius:
                        continue  # only the current ring
                    nx, ny = cx + dx, cy + dy
                    if is_wall(nx, ny):
                        continue
                    d = math.hypot(nx + 0.5 - x, ny + 0.5 - y)
                    if d < best_d:
                        best_d = d
                        best = (nx + 0.5, ny + 0.5)
            if best is not None:
                return best

        return (x, y)
