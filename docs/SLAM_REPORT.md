# SLAM / Localization Development Report

A chronological account of how the localization system in this simulator was
built up — every method attempted, the problem it ran into, and the fix —
ending with the accumulated final architecture.

---

## 0. Context: what the simulator gives us

Several design facts about the sim shaped every decision, so they're worth
stating up front:

- **The robot has a true (continuous) pose and a *believed* pose.** Planning and
  exploration run on the belief; actuation (movement, collision) happens at the
  true pose. Localization's whole job is to keep the belief close to the truth.
- **Mapping is kept world-aligned and decoupled.** The occupancy grid is built
  from observations in absolute cell coordinates, so the map stays correct
  regardless of localization error. This was a deliberate choice so the *other*
  subsystems (exploration, planning, rendering) remain testable in isolation.
  It also means this is, strictly, **Monte Carlo Localization against a map
  (AMCL)** rather than full coupled graph-SLAM.
- **Sensor noise flips a cell's occupied/free *flag*, but not *which* cells are
  observed.** This single fact made the early scan-matching ideas viable.
- **`exact` localization** (perfect dead-reckoning) is always available as a
  zero-error baseline for testing everything downstream.

The relevant files:

| File | Role |
|------|------|
| `localization/slam.py` | the localizer (became an MCL particle filter) |
| `localization/odometry.py`, `exact.py` | drift-only and ground-truth baselines |
| `sensing/sensor_model.py` | cell FOV for mapping + continuous range scan for localization |
| `sensing/fov.py` | shared recursive-shadowcasting field of view |
| `agents/base_agent.py` | occupancy mapping, localization wiring, stuck recovery |
| `planning/astar.py` | planner (gained a recovery mode) |
| `core/agent.py`, `config.py` | factory + tunables |

---

## 1. Starting point: the wall-anchoring heuristic

**Method.** The original `SLAMLocalization` predicted a pose from odometry, then
"corrected" it by nudging toward observed wall cells and away from free cells —
a hand-rolled anchoring rule.

**Issue.** The anchoring had no relationship to the true position. It pulled the
estimate toward whatever walls were nearby, so error *accumulated* rather than
being corrected. It "struggled with sensor noise and failed with odometry
drift" — because it never actually used the scan to infer where the robot was.

**Fix.** Discard it entirely and build a real scan-matching localizer
(Section 2 onward).

---

## 2. Scan matching v1: most-surrounded cell

**Method.** Predict from odometry, then over a small window of candidate cells
around the prediction, pick the observed cell whose 3×3 neighbourhood is most
fully observed (the robot's own cell is the most "surrounded"), and fuse with a
complementary filter. Key insight: the *set* of observed cell coordinates is
noise-immune, so this ignores false positives/negatives by construction.

**Issue.** It barely used the map structure — in open areas many cells look
equally surrounded, so the correction was weak and the residual error sat around
a full cell. It also didn't use the obvious physical constraint that the robot
**cannot be inside a wall**.

**Fix.** Two upgrades (Section 3).

---

## 3. Scan matching v2: FOV correlation + free-space constraint

**Method.**
- For each candidate cell near the prediction, compute what the robot *would*
  see from there via **recursive shadowcasting over its own internal map**, and
  score it by Jaccard overlap with the actual observation. Extracted the
  shadowcasting into a shared `sensing/fov.py` used by both the sensor and the
  localizer.
- Added a **hard free-space constraint**: if the estimate lands in a known wall,
  snap it to the nearest free cell.

**Issue.** Worked well (≈0.2–0.5 cell error, 0 % in walls), but the user's config
had the fusion gain set to `0`, which silently disabled the correction entirely
(reducing SLAM to drift + wall-projection, 8–10 cell error).

**Fix.** Restored a working gain (0.5) and flagged that `gain=0` disables the
scan match. This version localized to ~0.3 cells under 0.3 drift + 10 % noise
while plain odometry diverged to 14 cells.

---

## 4. The real thing: Monte Carlo Localization (particle filter / AMCL)

**Method.** Replaced the single-estimate scan matcher with a proper particle
filter — the algorithm behind ROS `amcl`:

1. **Predict** — propagate every particle through the odometry motion model with
   per-particle noise.
2. **Weight** — a **likelihood field**: a chamfer distance-transform of known
   obstacles; each particle's projected obstacle hits are scored by a Gaussian on
   distance-to-nearest-wall, mixed with a uniform floor (`z_rand`) for outlier
   robustness. Particles inside known walls are down-weighted.
3. **Resample** — systematic (low-variance) resampling on effective-sample-size
   collapse, with roughening.
4. **Estimate** — weighted mean of the cloud (continuous, sub-cell).

The agent was changed to feed **robot-frame relative measurements** (what a real
range sensor outputs) instead of absolute world coordinates, so the filter has
to *infer* the absolute pose. A particle-cloud render layer was added.

**Issue A — divergence with no recovery.** On some maps (e.g. an open cave) the
cloud lost lock and never recovered (5.7 cell error) — the classic
kidnapped-robot problem.

**Fix A — Augmented MCL.** Track slow/fast running averages of measurement fit;
when recent fit drops below the long-run fit, inject random hypotheses (sampled
from known-free space) on resampling to recover.

**Issue B — perceptual aliasing.** One cave seed still diverged because a *wrong*
pose explained the wall endpoints just as well as the right one, so the fit
stayed high and recovery never triggered.

**Fix B — free-space negative evidence.** Also score cells seen as *free*:
if a free reading would fall on a known wall from a candidate pose, penalise it.
This breaks aliasing, since a wrong pose puts free space where obstacles
actually are. After this, all room/bsp/cave seeds localized to 0.2–0.5 cells.

---

## 5. Estimate: weighted mean → densest cluster (mode)

**Method.** The reported pose was the weighted mean of all particles.

**Issue.** When belief goes bimodal (two candidate locations), the mean lands in
the empty gap *between* the clusters — a pose neither hypothesis actually claims.

**Fix.** Switched to a **densest-cluster (mode)** estimate: bin particle weight
per cell, take the modal cell, and return the weighted centroid of that cell's
neighbourhood. It commits to the dominant hypothesis instead of averaging across
both. (Improved the trickiest aliasing seed and never regressed the unimodal
cases.)

---

## 6. Richer sensor error model

**Method.** Added two independent range-finder error modes alongside the
existing false-positive / false-negative occupancy flips:
- **`range_sigma`** — Gaussian noise on the measured distance to each hit.
- **`range_outlier_rate`** — chance a reading is a gross outlier (specular
  reflection / mixed pixel) at a random distance.

**Result.** Demonstrated the filter's robustness: with σ = 2 cells + 10 %
outliers it still held ~0.3 cell error and reached the goal, while odometry was
hopeless. (Also exposed and motivated the Section 7 fix.)

---

## 7. The big one: localization was tile-quantized

**Issue.** Localization built its scan from `int(true_position)` — the *tile*,
not the continuous pose. So the sub-tile position (14.7 vs 14.2) never affected
the measurements, and **no algorithm could resolve position better than a whole
tile**. The "continuous" estimate was really just tile-centre + dead-reckoning.

**Fix.** A genuine continuous range finder:
- **`range_scan()`** casts rays (DDA grid traversal) from the **floating-point**
  pose, returning **sub-cell hit distances**. The ranges now depend on exactly
  where in the tile the robot is.
- The measurement model reads a **bilinearly-interpolated** distance field, so
  the particle likelihood varies *smoothly* with sub-cell position — enabling
  within-tile resolution.
- Range noise moved onto the continuous ranges (where a real range finder's
  noise lives), and free-space evidence is taken from the *true* range so an
  overshooting reading can't fabricate evidence against the true pose (a
  divergence bug found and fixed here).

**Result.** Continuous error consistently below tile-snapped error (BSP:
0.18 vs 0.30), with the belief tracking the true pose in smooth sub-cell steps.

---

## 8. "Struggles in corridors"

**Issue.** Along a straight corridor every position looks identical to a range
sensor, so the *along-corridor* axis is unobservable. The filter was injecting
isotropic noise every step (motion-model process noise **+** resampling
roughening); along that unobservable axis the noise was never corrected, so the
estimate random-walked down the corridor and sometimes lost lock entirely
(15.6 cell error on one room map).

**Fix.** Since the sim feeds essentially exact motion, the heavy process noise
was unjustified:
- Decoupled the filter's process noise from `odometry_noise` into its own small
  `slam_motion_sigma` (0.02).
- Reduced roughening (0.12 → 0.02).
- No process noise while stationary.

**Result.** That room map went from 15.6 → 0.23 cell error; rooms/bsp/cave all
solid (0.18–0.30). **Honest limit:** a pure `maze` is *globally* aliased (every
1-wide corridor cell looks like dozens of others), which no noise setting fixes —
the fundamental hard case for single-scan localization.

---

## 9. "Gets stuck in a corner and won't move"

**Issue.** Two distinct deadlocks, both surfacing on mazes:
- **Mode A — phantom-wall seal.** A single false-positive reading marked a
  corridor cell as a wall; that sealed off the known-free region; once the robot
  moved past it the bad cell was out of line-of-sight and never re-observed. The
  frontier search couldn't escape → no target → frozen, *not even planning*.
  This happened **even with perfect localization**.
- **Mode B — wrong pose.** SLAM divergence in an aliased maze → planning from a
  wrong cell → following an un-followable path → no movement.

**Fix.** A phase-independent **stuck detector + recovery** in the agent:
- `stuck_steps` counts no-progress steps; after a limit (25) recovery fires.
- **Re-open boundary** — reset the believed-walls bounding the reachable region
  to "unknown"; real walls snap back on the next sense, the phantom seal stays
  open (fixes Mode A).
- **Physical twitch** — take one real step into a sensed-free neighbour to break
  a physical deadlock and generate fresh observations to relocalize from (fixes
  Mode B). Plus a planner `allow_walls` recovery mode and a frontier
  `nearest_unknown` fallback.

**Result.** Worst freeze across all maps dropped from *permanent* to ≤ 514 steps;
mazes that never finished now complete.

---

## 10. Attacking the root cause: evidence-accumulating occupancy grid

**Issue.** The phantom walls in Section 9 existed because the occupancy update
was too aggressive: `±0.2` per observation with a `0.6` wall threshold meant a
**single** false-positive reading flipped a cell straight to "wall."

**Fix (a log-odds-style model).**
- **Smaller step** (`map_update_step` = 0.1): a cell now needs two consistent
  readings to cross a threshold and four to saturate, so one false reading just
  nudges it back toward "unknown" and is corrected by the next true reading.
- **Locking**: once a cell saturates past `map_lock_high` (0.9 → wall) or
  `map_lock_low` (0.1 → free) it is frozen and never updated again — immune to
  later noise bursts; the map stops flickering.
- **Recovery unlocks everything** as a safety net, so a wrongly-locked cell can
  never keep the robot trapped (locks re-establish as evidence re-accumulates).

**Result.** Phantom-wall recoveries dropped to **zero** on rooms/bsp/cave;
**every** map (including all maze seeds) now finishes; worst freeze ≤ 25 steps;
localization error held or improved.

---

## Accumulation of the fixes — the final architecture

The localizer is now a **Monte Carlo Localization particle filter (AMCL-class)**
localizing against an **evidence-accumulating occupancy grid**, fed by a
**continuous range finder**, wrapped in **divergence/stuck recovery**:

**Sensing (`sensing/`)**
- Recursive-shadowcasting FOV with true line-of-sight occlusion (no seeing
  through walls or around corners).
- Two outputs: a **cell FOV** for mapping, and a **continuous ray scan**
  (DDA, sub-cell ranges) for localization.
- Error models: false-positive, false-negative, Gaussian range noise, gross
  range outliers.

**Mapping (`agents/base_agent.py`)**
- Small-step occupancy updates (robust to single false readings) + **cell
  locking** at saturation. World-aligned and decoupled so other systems stay
  testable.

**Localization (`localization/slam.py`)**
- Predict: odometry motion model, small process noise, none while stationary.
- Weight: bilinear likelihood field (sub-cell), free-space negative evidence,
  in-wall penalty, `z_rand` outlier floor.
- Resample: systematic + roughening; **Augmented-MCL** random injection on fit
  collapse for kidnapped-robot recovery.
- Estimate: **densest-cluster (mode)**, projected out of walls.
- Fed **robot-frame relative measurements** (must infer absolute pose).

**Recovery (`agents/base_agent.py`, `planning/astar.py`, `exploration/frontier.py`)**
- Stuck detector → unlock all cells, re-open boundary walls, physical twitch
  step; planner `allow_walls` mode and `nearest_unknown` fallback.

### Results evolution (localization error vs. odometry, feature-rich maps)

| Stage | Typical error | Notes |
|-------|---------------|-------|
| Wall-anchoring heuristic | diverges | no real correction |
| FOV scan match + free constraint | ~0.3 cell | gain must be > 0 |
| Particle filter (MCL) | ~0.2–0.5 cell | matched `exact` at low noise |
| + Augmented MCL + free evidence | 0.2–0.5 across all seeds | recovers from divergence/aliasing |
| + continuous range scan | sub-tile (0.18 vs 0.30 tiled) | resolves within a tile |
| + corridor tuning | room 15.6 → 0.23 | low process noise |
| + small-step / locking occupancy | held/improved; **all maps finish** | phantom seals eliminated |

(For reference, plain `odometry` under the same drift+noise diverges to
10–25 cells and ends up inside walls 60–90 % of the time.)

### Known limitations
- **Pure mazes** are globally aliased; localization error sits around ~1 cell and
  the filter can briefly wander before recovery — fundamental to single-scan
  localization without loop closure / distinctive landmarks. The robot still
  explores and completes because the map stays clean and it never permanently
  freezes.
- This is **localization against a map**, not coupled graph-SLAM. There is no
  pose-graph back-end or loop closure; global drift is bounded by the
  (consistent) map rather than by optimisation.

### Key tunables (`config.py`)
```
localization            "odometry" | "slam" | "exact"
slam_motion_sigma       0.02   particle process noise (low → tight corridors)
slam_num_particles      200
slam_measurement_sigma  1.2    likelihood-field std-dev (cells)
slam_z_rand             0.1    outlier floor
sensor_num_beams        72     rays for the range scan
sensor_range_sigma      range-finder Gaussian noise
sensor_range_outlier_rate  gross-outlier chance
map_update_step         0.1    occupancy evidence step (small → noise-robust)
map_lock_high / low      0.9 / 0.1   saturation lock thresholds
```
