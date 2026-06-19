"""
sensor_model.py

Two complementary outputs from one sensor:

1. sense()       - cell field-of-view for MAPPING. Recursive shadowcasting over
                   the grid gives the visible cells (occluded by walls); each is
                   reported with its (noisy) occupancy. Resolution is per-cell,
                   which is exactly what an occupancy grid needs.

2. range_scan()  - continuous range finder for LOCALIZATION. Rays are cast from
                   the agent's *floating-point* position and return sub-cell hit
                   distances. This is what lets SLAM localize WITHIN a tile: the
                   measured ranges depend on where the robot actually is, not just
                   which tile it occupies.

Error models (all independent, default off):
  - false_positive_rate : a free cell is reported as an obstacle (mapping)
  - false_negative_rate : an obstacle is missed (mapping + a dropped ray)
  - range_sigma         : Gaussian error on each measured range (range finder)
  - range_outlier_rate  : chance a ray is a gross outlier (specular / mixed pixel)
"""

import math
from typing import List, Tuple

from sensing.fov import visible_cells

Position = Tuple[float, float]
Observation = Tuple[int, int, bool]
# Relative range-scan reading: (dx, dy) float offset from the sensor, occupied?
ScanReading = Tuple[float, float, bool]


class SensorModel:
    def __init__(
        self,
        sensor_range: float,
        mode: str = "radius",
        false_positive_rate: float = 0.0,
        false_negative_rate: float = 0.0,
        range_sigma: float = 0.0,
        range_outlier_rate: float = 0.0,
        num_beams: int = 72,
        rng_manager=None,
    ):
        self.sensor_range = sensor_range
        self.mode = mode

        self.false_positive_rate = false_positive_rate
        self.false_negative_rate = false_negative_rate
        self.range_sigma = range_sigma
        self.range_outlier_rate = range_outlier_rate
        self.num_beams = num_beams

        self.rng = rng_manager.behaviour_rng() if rng_manager else None

        # FOV cache: the shadowcast visible-cell SET depends only on the agent's
        # integer cell and the (static) true map, but the agent crosses a cell only
        # every ~5 steps, so it is recomputed far more than it changes. Cache it
        # keyed on (cx, cy, radius); the per-cell occupancy + noise still runs each
        # step, so observations are bit-identical - only the recursion is skipped.
        self._fov_key = None
        self._fov_cells = None

    # =========================
    # Mapping: cell field-of-view
    # =========================

    def sense(self, environment, position: Position) -> List[Observation]:
        """
        Occupancy of every cell currently visible to the agent (for mapping).
        Cells hidden behind walls (shadowed) are not reported.
        """
        cx, cy = int(position[0]), int(position[1])
        radius = int(self.sensor_range)

        observations = []
        for x, y in self._visible_cells(environment, cx, cy, radius):
            true_occ = not environment.is_free(x, y)

            if true_occ:
                # Missed detection -> report as free.
                reported = not (self.rng is not None and self.rng.random() < self.false_negative_rate)
                observations.append((x, y, reported))
            else:
                # Spurious detection -> free cell reported as an obstacle.
                spurious = self.rng is not None and self.rng.random() < self.false_positive_rate
                observations.append((x, y, spurious))

        return observations

    def _visible_cells(self, environment, cx, cy, radius) -> set:
        """Cells visible from (cx, cy) within radius, occluded by walls.

        Cached on the integer cell (the true map is static); recomputed only when
        the agent's cell changes. The predicates read the grid directly with local
        bounds checks, avoiding the environment->map->in_bounds wrapper chain that
        the profile showed dominating (millions of calls)."""
        key = (cx, cy, radius)
        if key == self._fov_key:
            return self._fov_cells

        m = environment.map
        grid, w, h = m.grid, m.width, m.height
        cells = visible_cells(
            is_blocked=lambda x, y: not (0 <= x < w and 0 <= y < h and grid[y][x] == 0),
            in_bounds=lambda x, y: 0 <= x < w and 0 <= y < h,
            cx=cx, cy=cy, radius=radius,
        )
        self._fov_key = key
        self._fov_cells = cells
        return cells

    # =========================
    # Localization: continuous range scan
    # =========================

    def range_scan(self, environment, position: Position) -> List[ScanReading]:
        """
        Cast num_beams rays from the CONTINUOUS position and return relative
        readings (dx, dy, occupied) as floats.

        Obstacle hits carry sub-cell geometry (the range depends on the exact
        position within the tile), and each ray also contributes a free reading
        partway along it as negative evidence.
        """
        ox, oy = position
        max_r = float(self.sensor_range)
        scan: List[ScanReading] = []

        for i in range(self.num_beams):
            ang = 2.0 * math.pi * i / self.num_beams
            ux, uy = math.cos(ang), math.sin(ang)

            hit_range = self._cast_ray(environment, ox, oy, ux, uy, max_r)

            # No obstacle in range, or a missed detection -> free out to max range.
            missed = self.rng is not None and self.rng.random() < self.false_negative_rate
            if hit_range is None or missed:
                scan.append((ux * max_r, uy * max_r, False))
                continue

            # Free space the ray passed through is reliable geometry, so derive
            # it from the TRUE range; only the obstacle endpoint carries range
            # noise (otherwise an overshooting reading would mark a real wall as
            # free and create false evidence against the true pose).
            if hit_range > 1.0:
                fr = hit_range * 0.5
                scan.append((ux * fr, uy * fr, False))           # free space before it

            r = self._noisy_range(hit_range, max_r)
            scan.append((ux * r, uy * r, True))                  # obstacle endpoint

        return scan

    def _cast_ray(self, environment, ox, oy, ux, uy, max_r):
        """
        DDA grid traversal from a continuous origin. Returns the distance to the
        first obstacle surface, or None if nothing is hit within max_r.

        Reads the true grid directly with inline bounds checks (the profile showed
        the environment.is_free -> map.is_free -> in_bounds chain, called once per
        cell per beam, was a top cost). Behaviour is identical: out-of-bounds = no
        hit (None), a wall cell = hit; the origin being wall/OOB returns 0.0.
        """
        m = environment.map
        grid, w, h = m.grid, m.width, m.height

        cx, cy = int(ox), int(oy)
        if not (0 <= cx < w and 0 <= cy < h and grid[cy][cx] == 0):
            return 0.0

        if ux != 0:
            step_x = 1 if ux > 0 else -1
            bx = (cx + 1) if ux > 0 else cx
            t_max_x = (bx - ox) / ux
            t_delta_x = abs(1.0 / ux)
        else:
            step_x = 0
            t_max_x = float("inf")
            t_delta_x = float("inf")

        if uy != 0:
            step_y = 1 if uy > 0 else -1
            by = (cy + 1) if uy > 0 else cy
            t_max_y = (by - oy) / uy
            t_delta_y = abs(1.0 / uy)
        else:
            step_y = 0
            t_max_y = float("inf")
            t_delta_y = float("inf")

        while True:
            if t_max_x < t_max_y:
                cx += step_x
                t = t_max_x
                t_max_x += t_delta_x
            else:
                cy += step_y
                t = t_max_y
                t_max_y += t_delta_y

            if t > max_r:
                return None
            if not (0 <= cx < w and 0 <= cy < h):
                return None
            if grid[cy][cx] != 0:
                return t

    def _noisy_range(self, r, max_r):
        """Apply range-finder noise to a measured distance."""
        if self.rng is None:
            return r
        if self.range_outlier_rate > 0 and self.rng.random() < self.range_outlier_rate:
            return self.rng.uniform(0.0, max_r)
        if self.range_sigma > 0:
            return max(0.0, r + self.rng.gauss(0, self.range_sigma))
        return r
