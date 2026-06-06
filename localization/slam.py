"""
slam.py

Monte Carlo Localization (particle filter) - the real-world algorithm behind
ROS's `amcl`. Strictly this is localization against a map (the "L" in SLAM);
the map is built separately so the rest of the system can be tested with a
correct map regardless of localization error.

Pipeline each update (a recursive Bayes filter):
  1. PREDICT   - propagate every particle through the odometry motion model
                 with per-particle noise (represents wheel slip / drift).
  2. WEIGHT    - score each particle against the sensor scan using a likelihood
                 field: the agent reports range/bearing-style RELATIVE hits, we
                 project them into the world from each particle, and reward
                 particles whose hits land on known obstacles. A uniform floor
                 (z_rand) keeps it robust to false readings.
  3. RESAMPLE  - systematic resampling when the effective sample size collapses,
                 with roughening to avoid particle depletion.
  4. ESTIMATE  - the weighted mean of the cloud (a continuous, sub-cell pose).

Real-world fidelity:
  - Sensor data is in the robot frame (relative), so the filter never sees the
    true pose - it must infer it, like a real robot.
  - The likelihood field + z_rand floor is the standard AMCL beam-endpoint model.
  - Particles inside known walls are down-weighted (physically impossible), and
    the reported estimate is projected out of walls.

Compared with full graph-SLAM this omits the mapping back-end and loop closure;
because it localizes against an already-consistent map, global drift is bounded
without them.
"""

import math
import random
from typing import Tuple, List, Optional

Position = Tuple[float, float]
RelObservation = Tuple[int, int, bool]  # (dx, dy, occupied) in the robot frame

# Internal-map occupancy threshold above which a cell is treated as a wall.
_WALL_THRESHOLD = 0.6
_SQRT2 = math.sqrt(2.0)


class SLAMLocalization:
    def __init__(
        self,
        noise_sigma: float = 0.1,
        sensor_range: float = 5,
        num_particles: int = 200,
        measurement_sigma: float = 1.2,
        z_rand: float = 0.1,
        init_sigma: float = 0.5,
        rough_sigma: float = 0.02,
        max_endpoints: int = 24,
        alpha_slow: float = 0.01,
        alpha_fast: float = 0.2,
        rng_manager=None,
    ):
        """
        noise_sigma:       odometry motion-model noise std-dev (per step).
        sensor_range:      sensor range (unused directly; kept for parity/metadata).
        num_particles:     size of the particle cloud.
        measurement_sigma: likelihood-field std-dev (cells); larger = more forgiving.
        z_rand:            uniform-noise mixing weight for outlier robustness (0..1).
        init_sigma:        spread of the initial cloud around the start pose.
        rough_sigma:       jitter added on resampling to fight particle depletion.
        max_endpoints:     cap on scan endpoints scored per step (speed).
        """
        self.motion_sigma = noise_sigma
        self.sensor_range = int(sensor_range)
        self.num_particles = num_particles
        self.measurement_sigma = measurement_sigma
        self.z_rand = z_rand
        self.init_sigma = init_sigma
        self.rough_sigma = rough_sigma
        self.max_endpoints = max_endpoints

        # Augmented-MCL recovery: track slow/fast averages of measurement fit and
        # inject random hypotheses when the fit collapses (filter divergence /
        # kidnapped robot).
        self.alpha_slow = alpha_slow
        self.alpha_fast = alpha_fast
        self.w_slow = 0.0
        self.w_fast = 0.0
        self.injection_ratio = 0.0

        self.rng = rng_manager.behaviour_rng() if rng_manager else random.Random()

        self.particles: Optional[List[List[float]]] = None
        self.weights: Optional[List[float]] = None

    # =========================
    # Main API
    # =========================

    def update(
        self,
        believed_position: Position,
        motion: Position,
        scan: List[RelObservation],
        internal_map: Optional[List[List[float]]] = None,
    ) -> Position:
        # 1. Lazy init around the (known) start pose.
        if self.particles is None:
            self._init_particles(believed_position)

        # 2. Motion update.
        self._predict(motion)

        # 3. Measurement update (requires the map).
        if internal_map is not None:
            self._weight(scan, internal_map)
            self._resample_if_needed(internal_map)

        # 4. Report the weighted-mean pose, kept out of known walls.
        est = self._estimate()
        if internal_map is not None:
            est = self._project_out_of_walls(est, internal_map)
        return est

    # =========================
    # 1. Initialisation
    # =========================

    def _init_particles(self, center: Position):
        cx, cy = center
        self.particles = [
            [self.rng.gauss(cx, self.init_sigma), self.rng.gauss(cy, self.init_sigma)]
            for _ in range(self.num_particles)
        ]
        self.weights = [1.0 / self.num_particles] * self.num_particles

    # =========================
    # 2. Motion model
    # =========================

    def _predict(self, motion: Position):
        mx, my = motion
        # Only inject process noise while actually moving - diffusing the cloud
        # when stationary (or every step more than the motion warrants) lets the
        # estimate wander along unobservable directions, e.g. down a corridor.
        moving = (mx != 0.0 or my != 0.0)
        sigma = self.motion_sigma if moving else 0.0
        for p in self.particles:
            p[0] += mx
            p[1] += my
            if sigma > 0:
                p[0] += self.rng.gauss(0, sigma)
                p[1] += self.rng.gauss(0, sigma)

    # =========================
    # 3. Measurement model (likelihood field)
    # =========================

    def _weight(self, scan: List[RelObservation], internal_map):
        height = len(internal_map)
        width = len(internal_map[0])

        dist_field = self._distance_transform(internal_map, width, height)

        # Positive evidence: obstacle hits should land on map walls.
        hits = self._subsample([(dx, dy) for dx, dy, occ in scan if occ], self.max_endpoints)
        # Negative evidence: cells seen as free should NOT be map walls. This
        # breaks perceptual aliasing, where a wrong pose explains the walls just
        # as well but would put free space where obstacles actually are.
        frees = self._subsample([(dx, dy) for dx, dy, occ in scan if not occ], self.max_endpoints)

        two_sigma_sq = 2.0 * self.measurement_sigma * self.measurement_sigma
        log_floor = math.log(self.z_rand) if self.z_rand > 0 else -50.0
        log_free_conflict = math.log(0.35)  # penalty for free-cell-on-wall mismatch
        log_wall_penalty = math.log(1e-3)   # particle sitting inside a known wall

        n_terms = len(hits) + len(frees)

        log_weights = []
        fits = []  # absolute per-particle fit in [0, 1], for divergence detection
        for px, py in self.particles:
            score_lw = 0.0  # measurement fit only (no self-in-wall penalty)

            # Obstacle hits vs the (bilinearly-interpolated) likelihood field.
            # Continuous endpoints + continuous field => the score varies smoothly
            # with sub-cell particle position, so the filter resolves within a tile.
            for dx, dy in hits:
                d = self._sample_field(dist_field, px + dx, py + dy, width, height)
                if d is None:
                    prob = self.z_rand
                else:
                    prob = (1.0 - self.z_rand) * math.exp(-(d * d) / two_sigma_sq) + self.z_rand
                score_lw += math.log(prob) if prob > 0 else log_floor

            # Free readings that would fall inside a known wall are contradictions.
            for dx, dy in frees:
                ix, iy = int(px + dx), int(py + dy)
                if 0 <= ix < width and 0 <= iy < height and internal_map[iy][ix] > _WALL_THRESHOLD:
                    score_lw += log_free_conflict

            # Geometric-mean per-term likelihood: an absolute measure of how well
            # this particle explains the scan (1 = perfect, low = lost).
            fits.append(math.exp(score_lw / n_terms) if n_terms else 1.0)

            lw = score_lw
            # Physically impossible to sit inside a known wall.
            ipx, ipy = int(px), int(py)
            if not (0 <= ipx < width and 0 <= ipy < height) or internal_map[ipy][ipx] > _WALL_THRESHOLD:
                lw += log_wall_penalty

            log_weights.append(lw)

        # Normalise from log-space (subtract max for stability).
        max_lw = max(log_weights)
        raw = [math.exp(lw - max_lw) for lw in log_weights]
        total = sum(raw)
        if total <= 0:
            self.weights = [1.0 / self.num_particles] * self.num_particles
        else:
            self.weights = [w / total for w in raw]

        # Augmented MCL: update slow/fast fit averages and set injection ratio.
        # When recent fit (w_fast) drops below the long-run fit (w_slow), the
        # filter is likely lost, so inject random hypotheses on resample.
        if n_terms:
            w_avg = sum(fits) / len(fits)
            self.w_slow += self.alpha_slow * (w_avg - self.w_slow)
            self.w_fast += self.alpha_fast * (w_avg - self.w_fast)
            if self.w_slow > 0:
                self.injection_ratio = max(0.0, min(0.5, 1.0 - self.w_fast / self.w_slow))
            else:
                self.injection_ratio = 0.0

    def _distance_transform(self, internal_map, width, height):
        """Chamfer distance to the nearest known wall (the likelihood field)."""
        INF = float("inf")
        D = [
            [0.0 if internal_map[y][x] > _WALL_THRESHOLD else INF for x in range(width)]
            for y in range(height)
        ]

        # Forward pass
        for y in range(height):
            for x in range(width):
                if D[y][x] == 0.0:
                    continue
                best = D[y][x]
                if x > 0:
                    best = min(best, D[y][x - 1] + 1.0)
                if y > 0:
                    best = min(best, D[y - 1][x] + 1.0)
                if x > 0 and y > 0:
                    best = min(best, D[y - 1][x - 1] + _SQRT2)
                if x < width - 1 and y > 0:
                    best = min(best, D[y - 1][x + 1] + _SQRT2)
                D[y][x] = best

        # Backward pass
        for y in range(height - 1, -1, -1):
            for x in range(width - 1, -1, -1):
                best = D[y][x]
                if x < width - 1:
                    best = min(best, D[y][x + 1] + 1.0)
                if y < height - 1:
                    best = min(best, D[y + 1][x] + 1.0)
                if x < width - 1 and y < height - 1:
                    best = min(best, D[y + 1][x + 1] + _SQRT2)
                if x > 0 and y < height - 1:
                    best = min(best, D[y + 1][x - 1] + _SQRT2)
                D[y][x] = best

        return D

    def _sample_field(self, dist_field, x, y, width, height):
        """
        Bilinearly interpolate the cell-centred distance field at continuous
        world point (x, y). Cell (i, j) is centred at (i + 0.5, j + 0.5).
        Returns None if out of bounds or next to an unmeasured (infinite) cell.
        """
        fx, fy = x - 0.5, y - 0.5
        x0, y0 = math.floor(fx), math.floor(fy)
        x1, y1 = x0 + 1, y0 + 1

        if x0 < 0 or y0 < 0 or x1 >= width or y1 >= height:
            ix, iy = int(x), int(y)
            if 0 <= ix < width and 0 <= iy < height and not math.isinf(dist_field[iy][ix]):
                return dist_field[iy][ix]
            return None

        d00, d10 = dist_field[y0][x0], dist_field[y0][x1]
        d01, d11 = dist_field[y1][x0], dist_field[y1][x1]
        if math.isinf(d00) or math.isinf(d10) or math.isinf(d01) or math.isinf(d11):
            return None  # no nearby wall measured -> uninformative

        tx, ty = fx - x0, fy - y0
        top = d00 * (1 - tx) + d10 * tx
        bot = d01 * (1 - tx) + d11 * tx
        return top * (1 - ty) + bot * ty

    def _subsample(self, items, limit):
        if len(items) <= limit:
            return items
        step = len(items) / limit
        return [items[int(i * step)] for i in range(limit)]

    # =========================
    # 4. Resampling
    # =========================

    def _resample_if_needed(self, internal_map):
        # Resample when the cloud has collapsed (low effective sample size) or
        # when augmented-MCL wants to inject recovery particles.
        ess = 1.0 / sum(w * w for w in self.weights)
        if ess >= self.num_particles / 2.0 and self.injection_ratio <= 0.0:
            return

        n = self.num_particles
        free_cells = self._free_cells(internal_map) if self.injection_ratio > 0 else None
        new_particles = []

        # Systematic (low-variance) resampling.
        r = self.rng.random() / n
        c = self.weights[0]
        i = 0
        for m in range(n):
            # Augmented MCL: replace a fraction with random free-space hypotheses
            # to recover from divergence.
            if free_cells and self.rng.random() < self.injection_ratio:
                fx, fy = self.rng.choice(free_cells)
                new_particles.append([fx + 0.5, fy + 0.5])
                continue

            u = r + m / n
            while u > c and i < n - 1:
                i += 1
                c += self.weights[i]
            x, y = self.particles[i]
            new_particles.append([
                x + self.rng.gauss(0, self.rough_sigma),
                y + self.rng.gauss(0, self.rough_sigma),
            ])

        self.particles = new_particles
        self.weights = [1.0 / n] * n

        # Injected particles count as a reset; relax the fast average so we don't
        # keep injecting every step.
        self.w_fast = self.w_slow

    def _free_cells(self, internal_map):
        return [
            (x, y)
            for y, row in enumerate(internal_map)
            for x, p in enumerate(row)
            if p < 0.4
        ]

    # =========================
    # Estimate & constraints
    # =========================

    def _estimate(self) -> Position:
        """
        Densest-cluster (mode) estimate: find the cell holding the most particle
        weight, then return the weighted centroid of the particles in that cell
        and its immediate neighbours.

        This commits to the dominant hypothesis instead of averaging across the
        whole cloud, so when belief is split between two candidate locations the
        estimate snaps to the heavier cluster rather than landing in the empty
        gap between them.
        """
        # Accumulate weight per grid cell.
        bins = {}
        for w, (x, y) in zip(self.weights, self.particles):
            key = (int(x), int(y))
            bins[key] = bins.get(key, 0.0) + w

        if not bins:
            ex = sum(w * p[0] for w, p in zip(self.weights, self.particles))
            ey = sum(w * p[1] for w, p in zip(self.weights, self.particles))
            return (ex, ey)

        mx, my = max(bins, key=bins.get)

        # Weighted centroid of the dominant cluster (modal cell + 3x3 around it).
        sx = sy = sw = 0.0
        for w, (x, y) in zip(self.weights, self.particles):
            if abs(int(x) - mx) <= 1 and abs(int(y) - my) <= 1:
                sx += w * x
                sy += w * y
                sw += w

        if sw > 0:
            return (sx / sw, sy / sw)
        return (mx + 0.5, my + 0.5)

    def _project_out_of_walls(self, pos: Position, internal_map) -> Position:
        """If the estimate lands in a known wall, snap to the nearest free cell."""
        height = len(internal_map)
        width = len(internal_map[0])
        x, y = pos
        cx, cy = int(x), int(y)

        def is_wall(ix, iy):
            return (not (0 <= ix < width and 0 <= iy < height)
                    or internal_map[iy][ix] > _WALL_THRESHOLD)

        if not is_wall(cx, cy):
            return pos

        for radius in range(1, 5):
            best = None
            best_d = float("inf")
            for dy in range(-radius, radius + 1):
                for dx in range(-radius, radius + 1):
                    if max(abs(dx), abs(dy)) != radius:
                        continue
                    nx, ny = cx + dx, cy + dy
                    if is_wall(nx, ny):
                        continue
                    d = math.hypot(nx + 0.5 - x, ny + 0.5 - y)
                    if d < best_d:
                        best_d = d
                        best = (nx + 0.5, ny + 0.5)
            if best is not None:
                return best

        return pos

    # =========================
    # Introspection (for rendering / debugging)
    # =========================

    def get_particles(self) -> List[List[float]]:
        return self.particles if self.particles is not None else []
