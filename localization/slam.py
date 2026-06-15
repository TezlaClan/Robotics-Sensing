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

import numpy as np

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
        anchor_sigma: float = 1.0,
        jump_margin: float = 2.0,
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
        # Spread (cells) of an inter-agent relative measurement, used when a
        # confident neighbour anchors this filter (swarm SLAM).
        self.anchor_sigma = anchor_sigma

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

        # Cached distance field (likelihood field), keyed by the map version it
        # was built from, so it is recomputed only when the map actually changes.
        self._dist_field: Optional[np.ndarray] = None
        self._wallmask: Optional[np.ndarray] = None  # cached boolean wall mask
        self._df_version = None

        # Spurious-jump gating. The robot's pose can move at most ~|motion| per
        # step, so an estimate that leaps much further is a perceptual-aliasing
        # teleport, not real motion. jump_margin is the slack (cells) allowed
        # beyond |motion| for legitimate sub-cell correction before a jump is
        # rejected. Tracks the last accepted estimate and a run-length of
        # consecutive rejections (a genuine relocalization is accepted once it
        # persists, so the gate can't lock the filter out forever).
        self.jump_margin = jump_margin
        self.max_gated_steps = 40
        self._last_est: Optional[Position] = None
        self._gate_count = 0

    # =========================
    # Main API
    # =========================

    def update(
        self,
        believed_position: Position,
        motion: Position,
        scan: List[RelObservation],
        internal_map: Optional[List[List[float]]] = None,
        anchors: Optional[List[Tuple[Position, float]]] = None,
        map_version=None,
    ) -> Position:
        # 1. Lazy init around the (known) start pose.
        if self.particles is None:
            self._init_particles(believed_position)

        # 2. Motion update.
        self._predict(motion)

        # 3. Measurement update (requires the map).
        #    `anchors` are implied-pose hints from confident neighbours (swarm
        #    SLAM); they add evidence alongside the agent's own scan.
        #    `map_version` lets the distance field be cached across steps.
        if internal_map is not None:
            self._weight(scan, internal_map, anchors, map_version)
            self._resample_if_needed(internal_map)

        # 4. Report the mode pose, kept out of known walls, then reject spurious
        #    teleports (the map is correct - only the estimate jumped).
        est = self._estimate()
        if internal_map is not None:
            est = self._project_out_of_walls(est, internal_map)
            est = self._gate_jump(est, motion, internal_map)
        self._last_est = est
        return est

    # =========================
    # Spurious-jump gating
    # =========================

    def _gate_jump(self, est, motion, internal_map):
        """
        Reject perceptual-aliasing teleports. Real motion moves the pose at most
        ~|motion| per step, so an estimate that jumps much further is the filter
        committing to a look-alike location, not the robot moving. When that
        happens we distrust the scan-based estimate for this step, fall back to
        the motion-predicted pose (dead reckoning from the last good estimate),
        and re-seed the particle cloud there so it re-locks against the (correct,
        untouched) map - no map rebuild, no lost exploration.

        A run of rejections is capped (max_gated_steps): if the filter keeps
        insisting on the far location it is eventually accepted, so a genuine
        relocalization is not blocked forever.
        """
        if self._last_est is None:
            return est

        predicted = (self._last_est[0] + motion[0], self._last_est[1] + motion[1])
        jump = math.hypot(est[0] - predicted[0], est[1] - predicted[1])
        allowed = math.hypot(motion[0], motion[1]) + self.jump_margin

        if jump > allowed and self._gate_count < self.max_gated_steps:
            self._gate_count += 1
            self._reseed(predicted)
            return self._project_out_of_walls(predicted, internal_map)

        self._gate_count = 0
        return est

    def _reseed(self, center: Position):
        """Collapse the cloud back to a tight blob around `center` and stop the
        divergence-injection that caused the jump, so the filter re-converges
        from continuity instead of from the spurious alias."""
        cx, cy = center
        sigma = max(self.init_sigma, 0.75)
        self.particles = [
            [self.rng.gauss(cx, sigma), self.rng.gauss(cy, sigma)]
            for _ in range(self.num_particles)
        ]
        self.weights = [1.0 / self.num_particles] * self.num_particles
        self.w_fast = self.w_slow          # cancel the augmented-MCL injection
        self.injection_ratio = 0.0

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

    # Distances at/above this are treated as "no nearby wall measured"
    # (uninformative -> the reading contributes only the z_rand floor). Only
    # occurs when the map has no walls at all (e.g. before any sensing).
    _DF_INF = 1e9
    # Max chamfer-propagation iterations. 12 cells is well past where the
    # likelihood is non-negligible (exp(-12^2/2/1.2^2) ~ 1e-22), so the field is
    # numerically identical to full convergence where it matters.
    _DF_CAP = 12

    def _weight(self, scan: List[RelObservation], internal_map, anchors=None, map_version=None):
        """
        Vectorised likelihood-field measurement update (numpy).

        Identical model to the original per-particle loop - obstacle hits scored
        against a bilinearly-interpolated distance field, free readings penalised
        where they fall on walls, an in-wall penalty, swarm anchors, log-sum-exp
        normalisation, and the Augmented-MCL fit tracking - but evaluated over all
        particles and scan endpoints at once.
        """
        # The measurement update depends on the map ONLY through the binary wall
        # mask (cell > _WALL_THRESHOLD): the distance field, the free-cell-on-wall
        # test, and the in-wall penalty all use it. So cache the boolean mask and
        # the distance field together, keyed by map_version (which the agent bumps
        # only when a cell crosses the wall threshold). On the common step where
        # values drift but no cell flips, this skips the array build AND the
        # transform entirely. Behaviour is identical to recomputing every step.
        if (self._dist_field is None or map_version is None
                or map_version != self._df_version):
            occ = np.asarray(internal_map, dtype=float) > _WALL_THRESHOLD  # (H, W)
            self._dist_field = self._distance_transform(occ)
            self._wallmask = occ
            self._df_version = map_version
        occ = self._wallmask
        df = self._dist_field
        height, width = occ.shape

        anchor_two_sigma_sq = 2.0 * self.anchor_sigma * self.anchor_sigma
        two_sigma_sq = 2.0 * self.measurement_sigma * self.measurement_sigma
        log_floor = math.log(self.z_rand) if self.z_rand > 0 else -50.0
        log_free_conflict = math.log(0.35)  # free-cell-on-wall mismatch
        log_wall_penalty = math.log(1e-3)   # particle inside a known wall

        # Positive evidence: obstacle hits should land on map walls.
        hits = self._subsample([(dx, dy) for dx, dy, occ in scan if occ], self.max_endpoints)
        # Negative evidence: cells seen as free should NOT be map walls (breaks
        # perceptual aliasing where a wrong pose still explains the walls).
        frees = self._subsample([(dx, dy) for dx, dy, occ in scan if not occ], self.max_endpoints)
        n_terms = len(hits) + len(frees)

        P = np.asarray(self.particles, dtype=float)          # (N, 2)
        N = P.shape[0]
        px, py = P[:, 0], P[:, 1]

        score = np.zeros(N)                                  # measurement fit only

        # --- Obstacle hits vs the bilinearly-interpolated likelihood field. ---
        if hits:
            Hh = np.asarray(hits, dtype=float)               # (Kh, 2)
            sx = px[:, None] + Hh[None, :, 0]                # (N, Kh)
            sy = py[:, None] + Hh[None, :, 1]
            prob = self._sample_prob(df, sx, sy, two_sigma_sq, width, height)
            logp = np.where(prob > 0, np.log(np.where(prob > 0, prob, 1.0)), log_floor)
            score += logp.sum(axis=1)

        # --- Free readings that fall inside a known wall are contradictions. ---
        if frees:
            Ff = np.asarray(frees, dtype=float)              # (Kf, 2)
            fix = (px[:, None] + Ff[None, :, 0]).astype(int)  # int() truncation
            fiy = (py[:, None] + Ff[None, :, 1]).astype(int)
            inb = (fix >= 0) & (fix < width) & (fiy >= 0) & (fiy < height)
            wall = inb & occ[np.clip(fiy, 0, height - 1), np.clip(fix, 0, width - 1)]
            score += wall.sum(axis=1) * log_free_conflict

        # Per-particle absolute fit (geometric-mean per-term likelihood).
        fits = np.exp(score / n_terms) if n_terms else np.ones(N)

        lw = score.copy()

        # Physically impossible to sit inside a known wall.
        ipx, ipy = px.astype(int), py.astype(int)
        inb = (ipx >= 0) & (ipx < width) & (ipy >= 0) & (ipy < height)
        in_wall = (~inb) | occ[np.clip(ipy, 0, height - 1), np.clip(ipx, 0, width - 1)]
        lw += in_wall * log_wall_penalty

        # Inter-agent anchors (added to the resampling weight only, NOT the fit,
        # so they don't mask our own divergence). When our own scan is
        # uninformative these dominate and the cloud snaps toward the confident
        # neighbour's implied pose (collaborative recovery).
        if anchors:
            for (ax, ay), conf in anchors:
                d2 = (px - ax) ** 2 + (py - ay) ** 2
                lw += conf * (-d2 / anchor_two_sigma_sq)

        # Normalise from log-space (subtract max for stability).
        max_lw = lw.max()
        raw = np.exp(lw - max_lw)
        total = raw.sum()
        if total <= 0:
            self.weights = [1.0 / self.num_particles] * self.num_particles
        else:
            self.weights = (raw / total).tolist()

        # Augmented MCL: update slow/fast fit averages and set injection ratio.
        # When recent fit (w_fast) drops below the long-run fit (w_slow) the
        # filter is likely lost, so inject random hypotheses on resample.
        if n_terms:
            w_avg = float(fits.mean())
            self.w_slow += self.alpha_slow * (w_avg - self.w_slow)
            self.w_fast += self.alpha_fast * (w_avg - self.w_fast)
            if self.w_slow > 0:
                self.injection_ratio = max(0.0, min(0.5, 1.0 - self.w_fast / self.w_slow))
            else:
                self.injection_ratio = 0.0

    def _distance_transform(self, occ: np.ndarray) -> np.ndarray:
        """
        Chamfer distance (orthogonal step 1, diagonal step sqrt(2)) to the
        nearest wall, computed as a vectorised min-of-shifted-neighbours
        (Bellman-Ford on the chamfer mask) iterated to convergence. This is the
        same metric the old two-pass chamfer produced, so the likelihood field is
        numerically identical - just computed with numpy instead of Python loops.
        Converges in O(grid diameter) sweeps; we cap at width+height and break
        early once stable.
        """
        INF = self._DF_INF
        D = np.where(occ, 0.0, INF)
        if not occ.any():
            return D  # no walls measured yet -> all uninformative

        # Only distances up to a few measurement sigmas affect the likelihood
        # (beyond ~10 cells exp(-d^2/2sigma^2) < 1e-15), so iterating past that is
        # wasted: cap propagation there. Identical likelihood to full convergence,
        # but bounds the iteration count in open maps. Early-break when stable.
        max_iters = min(self._DF_CAP, D.shape[0] + D.shape[1])
        for _ in range(max_iters):
            best = D.copy()
            # orthogonal neighbours (+1)
            best[1:, :] = np.minimum(best[1:, :], D[:-1, :] + 1.0)
            best[:-1, :] = np.minimum(best[:-1, :], D[1:, :] + 1.0)
            best[:, 1:] = np.minimum(best[:, 1:], D[:, :-1] + 1.0)
            best[:, :-1] = np.minimum(best[:, :-1], D[:, 1:] + 1.0)
            # diagonal neighbours (+sqrt2)
            best[1:, 1:] = np.minimum(best[1:, 1:], D[:-1, :-1] + _SQRT2)
            best[1:, :-1] = np.minimum(best[1:, :-1], D[:-1, 1:] + _SQRT2)
            best[:-1, 1:] = np.minimum(best[:-1, 1:], D[1:, :-1] + _SQRT2)
            best[:-1, :-1] = np.minimum(best[:-1, :-1], D[1:, 1:] + _SQRT2)
            if np.array_equal(best, D):
                break
            D = best
        return D

    def _sample_prob(self, df, x, y, two_sigma_sq, width, height):
        """
        Vectorised bilinear interpolation of the cell-centred distance field at
        continuous points (x, y) -> hit probability with the z_rand floor. Cell
        (i, j) is centred at (i + 0.5, j + 0.5). Points out of bounds, or whose
        interpolation cell touches an unmeasured (>= _DF_INF) corner, get the
        z_rand floor (uninformative), matching the old scalar _sample_field.
        """
        thr = self._DF_INF * 0.1
        z = self.z_rand

        fx, fy = x - 0.5, y - 0.5
        x0 = np.floor(fx).astype(int)
        y0 = np.floor(fy).astype(int)
        x1, y1 = x0 + 1, y0 + 1

        # Primary case: the 2x2 bilinear cell is fully in bounds with all corners
        # measured -> interpolate.
        valid = (x0 >= 0) & (y0 >= 0) & (x1 < width) & (y1 < height)
        x0c, x1c = np.clip(x0, 0, width - 1), np.clip(x1, 0, width - 1)
        y0c, y1c = np.clip(y0, 0, height - 1), np.clip(y1, 0, height - 1)
        d00 = df[y0c, x0c]; d10 = df[y0c, x1c]
        d01 = df[y1c, x0c]; d11 = df[y1c, x1c]
        measured = (d00 < thr) & (d10 < thr) & (d01 < thr) & (d11 < thr)
        tx, ty = fx - x0, fy - y0
        top = d00 * (1 - tx) + d10 * tx
        bot = d01 * (1 - tx) + d11 * tx
        d_bil = top * (1 - ty) + bot * ty

        prob_bil = (1.0 - z) * np.exp(-(d_bil * d_bil) / two_sigma_sq) + z
        prob = np.where(valid & measured, prob_bil, z)

        # Fallback (only when the bilinear cell is out of bounds): the single
        # int(x), int(y) cell, if in bounds and measured. Mirrors the scalar
        # _sample_field so edge geometry is scored identically. Skipped entirely
        # in the common case where every point's bilinear cell is in bounds.
        oob = ~valid
        if oob.any():
            ix = x.astype(int)
            iy = y.astype(int)
            fb_inb = (ix >= 0) & (ix < width) & (iy >= 0) & (iy < height)
            d_fb = df[np.clip(iy, 0, height - 1), np.clip(ix, 0, width - 1)]
            fb_ok = oob & fb_inb & (d_fb < thr)
            prob_fb = (1.0 - z) * np.exp(-(d_fb * d_fb) / two_sigma_sq) + z
            prob = np.where(fb_ok, prob_fb, prob)
        return prob

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

    def confidence(self) -> float:
        """
        How sure the filter is of its pose, in [0, 1]. Combines:
          - cloud tightness: a concentrated cloud (small weighted spread) is
            confident; a diffuse one is not.
          - measurement fit (w_slow): the long-run scan-explanation quality, so a
            tight-but-wrong cloud (good geometry, bad fit) is NOT reported as
            confident and won't be trusted to anchor other agents.
        """
        if not self.particles or not self.weights:
            return 0.0

        mx = sum(w * p[0] for w, p in zip(self.weights, self.particles))
        my = sum(w * p[1] for w, p in zip(self.weights, self.particles))
        var = sum(
            w * ((p[0] - mx) ** 2 + (p[1] - my) ** 2)
            for w, p in zip(self.weights, self.particles)
        )
        spread = math.sqrt(max(0.0, var))
        spread_term = math.exp(-spread)          # ~1 when tight, ->0 when diffuse
        fit_term = max(0.0, min(1.0, self.w_slow))
        return max(0.0, min(1.0, spread_term * fit_term))
