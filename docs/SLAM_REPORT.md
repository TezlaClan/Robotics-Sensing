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

---

# Part II — Subsequent work (multi-agent, performance, robot-anchored mapping)

Everything above is the original single-agent localizer. This part records the
changes made afterward, in order, with the empirical results that drove each
decision — including the experiments that **failed**. Absolute numbers reflect
the configuration in effect when they were measured (notably `sensor_range`
changed from 3 to 8 partway through, which matters a lot for local mapping);
within each experiment the comparison is like-for-like. Sweeps are "3 agents,
`individual` maps, `comm_mode: local`, `swarm_slam` on, maps room/bsp/cave ×
seeds 1–20 = 60 runs" unless stated, reporting mission **completion**, final
**localization error** (cells, max over agents), and **map-warp** (cells where a
confident belief disagrees with ground truth).

## 11. Swarm SLAM — inter-agent pose anchoring

When agents are within range, a *confident* one anchors a *less-confident* one.
`SLAMLocalization.confidence()` = weighted cloud spread × the long-run
measurement fit `w_slow` (a tight, well-fitting cloud ≈ 1; a diffuse or lost
cloud ≈ 0; the `w_slow` factor stops a confidently-*wrong* cloud from anchoring
others). Each step an agent forms a noisy relative measurement of each in-range
peer that is more confident than itself, turns it into an implied pose
(`peer.believed − measured_offset`), and the filter folds it into the particle
weights (added to the resampling log-weight `lw`, **not** the fit, so it doesn't
mask the agent's own divergence).

Empirical (heavy-noise A/B, peak localization error over a run):
- bsp seed4 **4.28 → 1.83**, bsp seed8 2.14 → 1.69 — a lost agent that meets a
  confident peer is reined in.
- cave seed4 1.65 → 1.66, cave seed8 1.81 → 1.74 — essentially inert on
  feature-rich maps where each agent already localizes well.

So anchoring helps exactly where intended (divergent agent + confident neighbour)
and does nothing where it isn't needed. Tunable: `slam_anchor_sigma`.

## 12. Spurious-jump gating (teleport rejection)

**Symptom:** the believed pose occasionally teleports to a random *other* place
on the map and the agent gets briefly lost. **Cause:** the interaction of (a)
perceptual aliasing — a wrong location explains the scan as well as the truth;
(b) the Augmented-MCL random injection, which sprinkles candidate poses across
the whole map for kidnapped-robot recovery; and (c) the densest-cluster *mode*
estimate, which commits to whichever cluster is momentarily heaviest, turning the
ambiguity into a discrete jump rather than a blur.

**Fix (`_gate_jump` / `_reseed`):** the robot's pose can move at most ≈|motion|
per step, so an estimate that leaps more than `|motion| + slam_jump_margin` is a
teleport, not motion. Reject it: fall back to the motion-predicted pose
(dead-reckoning from the last good estimate) and re-seed the cloud there — the
**map is untouched**. A run of rejections is capped (`max_gated_steps = 40`) so a
genuine relocalization is eventually accepted.

Empirical (default-noise 60-run, before → after):
- shared: complete 51 → **58**, mean err 2.57 → **0.36**, max 31.75 → **1.16**,
  cells > 2 err: 9 → **0**.
- individual: complete 49 → 54, mean 2.66 → 0.40, max 31.19 → 1.42, > 2: 7 → 0.
- Heavy-noise A/B (gate off vs on): per-step belief jumps > 5 cells **401 → 0**;
  peak err mean 12.1 → 2.7; final-error max 15.4 → 1.9.
- Cost: 4.13 → 4.14 ms/step (a distance check; the reseed only runs on a gated
  step). Tunable: `slam_jump_margin`.

This is the single biggest accuracy win — the long error tail was almost entirely
teleports. Note the map is correct throughout, so the prior "unlock the map and
let it rebuild" recovery was treating the wrong thing.

## 13. Performance — vectorizing the particle filter

cProfile (3 agents, 400 steps ≈ 16.6 s) showed the filter was **94 %** of
runtime: `_weight` 14.9 s, dominated by 5.7 M `_sample_field` Python calls
(6.0 s) and a per-step chamfer `_distance_transform` (3.6 s). Threading was ruled
out (GIL; CPU-bound pure Python), so `_weight` was rewritten in numpy:

- All particles × scan endpoints scored at once; the distance transform replaced
  with a vectorized iterative-chamfer (min-of-shifted-neighbours).
- Verified **bit-exact** against the originals: distance transform max diff 0.0;
  the sampler 0.0 *after* discovering and replicating the original's
  out-of-bounds single-cell fallback (a real bug found in review — without it,
  near-border points wrongly got the `z_rand` floor); full single-step weight
  vector matched to **2.2 × 10⁻¹⁶** (machine epsilon).
- Speed: 6.56 s → 2.65 s / 400 steps (~2.5×).
- 60-run equivalence (old sequential vs vectorized): `individual` **bit-identical**
  (49/60, mean 2.66, median 0.35, p90 9.81); `shared` statistically equivalent
  (completion 51 vs 45 — float-summation order reorders which chaotic seeds
  complete, while median/tails match).

**Failed:** vectorizing `_estimate` (the mode binning) gave no speedup for 200
particles (numpy overhead offset the loop) and shifted a tie-break — reverted.

## 14. Performance — caching the distance field

The likelihood depends on the map *only* through the binary `> 0.6` wall mask, so
the field is cached and rebuilt only when that mask changes:

- First keyed on a per-value map version (~15 % hit rate during active mapping),
  then on a **wall-mask version** bumped only when a cell crosses the threshold —
  hit rate **15 % → 72 %**, distance-transform calls 767 → 250.
- **Bug found:** in `shared` map mode each agent had its own version, so an agent
  reused a stale field after a *peer* changed the shared grid — shared-mode mean
  err degraded 2.59 → 4.25. Fix: a shared version counter; this not only restored
  it but made `shared` match the original sequential filter **exactly** (51/60,
  2.57/0.40/7.79), removing a subtler staleness the per-value version had.
- Plus: capping the transform at 12 iterations (numerically identical — beyond
  ~12 cells the likelihood is < 1e-15), a `_sample_prob` fast-path skipping the
  OOB branch when all points are in bounds, and caching the numpy map view.
- Net: bsp 6.98 → 4.11 ms/sim-step; per-agent-step stays ≈ flat with swarm size
  (1.26 ms at 1 agent → 1.57 ms at 8), confirming linear scaling (comms/anchors
  are the only O(n²) piece and stay small).

## 15. Robot-anchored ("local") map mode

A `map_anchor` toggle: `world` integrates observations at their **true** cells
(map globally aligned to ground truth — the clean benchmark, default); `local`
re-anchors each observation to the robot's **believed** cell, so the map rides
the estimate like real SLAM and drifts with localization error. A `drift` render
layer visualizes the warp (magenta = a believed wall over truly-free space, cyan
= the wall's true cell vacated because it was placed elsewhere).

Empirical at the original `sensor_range: 3` — world 54/60 (loc 0.40, warp 0) vs
local **25/60** (loc 2.77, warp 43). The failure was diagnosed precisely: a
whole-frame integer shift is **unobservable** (the map co-drifts with the
belief, so the scan stays consistent), and the resulting offset makes planning
(believed frame) disagree with collision (true frame) → an agent gets stuck
oscillating one cell off a wall it believes is free. The stuck-recovery can't fix
it because it's a stable localization offset, not a phantom wall.

## 16. Trusted-anchor experiments — what did NOT work

The intended fix (anchor to the known start, chain outward through walls
confirmed while well-localized) was implemented and **empirically rejected** in
several forms:

| variant (local mode) | complete | loc mean | loc max | warp |
|---|--:|--:|--:|--:|
| baseline (all walls), range 3 | 25/60 | 2.77 | — | 43 |
| trusted-only likelihood, range 3 | 16/60 | 4.25 | — | 53 |
| correct-then-map reorder, range 3 | 24/60 | 3.95 | — | 54 |
| baseline, **range 8** | 56/60 | 0.59 | 8.18 | 17.6 |
| trusted-only, range 8 | 46/60 | 1.86 | 12.1 | 48 |
| additive trusted weight 0.2 | 56/60 | 0.67 | 4.96 | 21 |
| additive trusted weight 0.5 | 53/60 | 1.04 | 19.4 | 29 |
| additive trusted weight 1.0 | 46/60 | 1.66 | 26.4 | 38 |

Conclusions: (1) **Raising `sensor_range` 3 → 8 fixed local mode broadly** (25 →
56/60) — the baseline likelihood *already* includes locked walls as anchors, and
a larger FOV keeps them in view; the "anchor to known walls" intuition is
realized by the existing filter once it can see the anchors. (2) "Trusted-only"
is a strict *subset* of the walls baseline already uses, so it only removes
constraint → looser tracking → *more* drift. (3) An *additive* extra weight on
trusted-wall hits doesn't help either — the filter already counts those walls, so
re-weighting over-sharpens the weights (particle depletion); only a small weight
(0.2) trades worse average error for a lower worst-case. Kept as an off-by-default
`slam_trusted_weight` knob.

## 17. Map sharing across drifted frames — investigation

Reported symptom: in `local` mode, displaced phantom walls appear to "multiply
into locked error walls" when agents are nearby. The merge is a *confident-locked
copy* (a peer's locked cells are copied into cells the receiver hasn't locked,
and locked). Across differently-drifted local frames this index-merge can stamp
cross-frame phantom walls — but the data showed it is **not** the net cause:

| local-mode merge variant | complete | loc mean | loc max | locked phantom walls |
|---|--:|--:|--:|--:|
| merge on (current) | 56/60 | 0.59 | 8.2 | 3.2 |
| merge off | 55/60 | 0.91 | 6.6 | 4.6 |
| merge, copy-but-don't-lock | 55/60 | 0.75 | 13.0 | 2.8 |

Disabling or un-locking the merge does **not** help — merge is net-beneficial
(sharing correct geometry improves everyone's localization, which *reduces*
phantom walls). The phantom walls are mostly **self-inflicted** drift, frozen by
the permanent lock; the multi-agent appearance is partly the `drift` overlay
compositing every agent's displaced walls. No merge change was made.

## 18. Lock erosion — what DID work

Root cause from §17: a locked cell is frozen forever, so a displaced wall placed
during a drift is permanent even after localization recovers and the agent sees
free space there. **Fix (`lock_erosion`):** a locked cell observed
*contradicting* its locked state (a locked wall seen free, or a locked-free cell
seen as a wall) for `lock_erosion_patience` **consecutive** steps is unlocked and
reset to unknown so it can re-heal. The consecutive requirement makes stray
sensor noise harmless — a single false reading is undone by the next consistent
one (5 consecutive false-negatives ≈ 1e-5).

Empirical (60-run):
- `world` mode: **byte-identical** off vs on (60/60, mean 0.37, warp 0) — on a
  ground-truth-aligned map walls are essentially never contradicted, so erosion
  never fires. This is the safety property that lets it default on.
- `local` mode: loc mean 0.59 → **0.50**, **max 8.18 → 1.94** (4× lower
  worst-case drift), warp 17.6 → 14.3; completion 56 → 54 (within noise). The
  final locked-phantom count is ~flat (3.2 → 3.4) because *out-of-view* phantoms
  are never re-observed to erode — but the *harmful, re-observed* ones heal,
  which is why localization improves so much.
- Patience sweep (local): 3 → 50/60 (loc max 14.3, too eager); **5 → 54/60 (loc
  0.50 / max 1.94, best)**; 8 → 56/60 (loc max 8.85). Default **5**.

This is the first change in this round to actually improve local mapping — and it
acts on the **map** (lock permanence), not the filter; every localization-side
tweak (§16) had failed. Tunables: `lock_erosion`, `lock_erosion_patience`.

> Note: the 60-run figures in §18 are on the old seeds-1–20 set. §19 re-measures
> lock erosion on the new stratified hard set, where its effect is larger and the
> earlier "erosion looks net-negative" reading is shown to be selection bias —
> see `docs/TESTING.md`.

## 19. Search mode — escaping a sealing phantom wall

Symptom (local mode): a drift-placed phantom wall seals the way onward. The agent
oscillates against it — it keeps twitching, so the zero-motion `stuck_steps` timer
never fires, and it never deliberately re-visits the phantom to correct it. It
jitters in place indefinitely (or until `max_steps`).

Fix (`agents/base_agent.py`, `exploration/frontier.py`): detect the stall by **net
displacement of the believed cell over a window** *and* a **blocked-step fraction**
(the discriminator — a real seal is physically blocked most steps; an agent merely
maneuvering slowly is not, so healthy runs don't trip it). On a stall, enter
**search mode**: navigate to the **furthest reachable frontier** through known-free
space, sweeping the region's perimeter so its boundary walls — including the
phantom — are re-observed from new angles. Exit the moment a believed-wall is
cleared (so normal planning resumes onward); if nothing is reachable, fall back to
the existing hard recovery. Tunables: `search_recovery`, `search_window` (30),
`search_min_progress` (1.5 cells), `search_block_frac` (0.5).

Process notes / what was measured (this is also when the standard sweep was
overhauled — see `docs/TESTING.md`):

- On the **old 1–20 set** the feature was **invisible**: 56/60 with and without
  it. That set had become saturated (overfit), which is what prompted building a
  new harness (`tools/sweep.py`, parallel) and a fresh **stratified hard seed
  set** (30 runs). All numbers below are on that set, local mode, 3 agents.
- A first detector using net-displacement **alone** was too eager: it fired on
  healthy slow stretches and sent them on long surveys, breaking a run for each
  one it rescued (net zero). Adding the **blocked-step fraction** gate fixed this.
- **2×2 feature matrix** (vanilla = both recovery features off):

  | config | completion | loc mean | loc max | warp | steps |
  |---|---|---|---|---|---|
  | vanilla | 21/30 | 0.843 | 15.33 | 13.87 | 1803 |
  | + lock_erosion | 26/30 | 0.628 | 26.67 | 9.40 | 1159 |
  | + search_recovery | 21/30 | 0.557 | 13.67 | 13.78 | 1968 |
  | + both (default) | **27/30** | **0.447** | **12.09** | **8.80** | **1010** |

- **search alone is not a win** (21→21; it even hurts cave 6→4 — on an open,
  drift-prone map the survey wanders and times out). The result is the
  **interaction**: search re-observes the sealing phantom, and **lock erosion is
  what actually clears it**. Search on top of erosion: cave 7→8, **max drift
  26.7→12.1**, mean 0.63→0.45, warp 9.4→8.8, steps 1159→1010. Neither feature
  delivers this alone; both default on.
- `world` mode is unaffected in practice: with a ground-truth-aligned map the
  agent isn't sealed by phantoms, so the stall detector effectively never fires.

## 20. Search-mode refinements + multi-agent erosion protection

Three follow-ups after watching `local` runs (all measured on the standard hard
set, 30 runs, all features on; see `docs/TESTING.md`):

**(a) Linger at a survey target.** A single drive-by barely glimpsed a dead-end
before backtracking, so the sealing wall was rarely observed long enough to erode.
The agent now **dwells `search_linger` (6) steps** at a survey target (still
sensing) before moving on. A lingering step is explicitly *not* counted toward the
stuck timer or the blocked-step window — an early version did count it, which
spuriously tripped hard recovery and re-triggered search the instant the dwell
ended (it regressed the set to 23/30 with ~100k boundary reopens until fixed).

**(b) Give up on real walls.** Search marks a target it lingered at without opening
a wall as *visited* and won't revisit it that episode; when every reachable
frontier is exhausted it exits search instead of battering a genuine (non-erodible)
wall forever. This is what stops the "stuck searching an actual wall" behaviour.

**(c) Multi-agent erosion protection (merge interaction).** Symptom reported: with
several agents in an area a wall is *harder to erode*. Mechanism confirmed in
`communication_model._merge_maps`: the confident-locked merge copies a peer's
locked cells onto any cell we have not locked. After we erode a phantom (unlock →
0.5), a peer that still holds it locked **re-imposes and re-locks the wall on the
next merge**, undoing our erosion.

But a **blanket** protection of every recently-eroded cell made things *worse*:

| erosion-protect | completion | loc mean | loc max | warp | recoveries | reopens |
|---|---|---|---|---|---|---|
| off | 27/30 | 0.436 | 10.10 | 7.91 | 410 | 32.8k |
| blanket (30 steps) | **23/30** | 0.630 | 12.35 | 12.64 | 956 | 100k |
| **targeted (30 steps)** | **27/30** | **0.388** | **5.32** | **7.17** | **300** | 32.0k |

The blanket version shields *bad* erosions too — when an agent wrongly erodes a
**real** wall, blocking the peer from restoring it leaves a hole the agent drives
into, doubling recoveries and reopens. The fix is **targeted**: refuse a peer's
wall on an eroded cell only while *our own current value still reads free*
(`< 0.4`). A genuine phantom (we keep sensing free) stays protected — the reported
symptom — while a misfired erosion (we re-sense wall) is left for the peer to
correct. Targeted protection gives the best numbers of the whole effort:
worst-case drift 10.1→**5.3**, jump-gates 293→**30**. Tunable:
`erosion_protect_steps` (0 disables).

## 21. Locked-only navigation + merge conflict reconsideration

Two ideas aimed at the hard set's seal/thrash failures (all on the all-features-on
hard set, 30 runs; baseline = both new flags off = 14/30).

**(a) Locked-only navigation (`nav_locked_only`, default ON).** The A* planner
previously blocked on any `>0.6` cell — i.e. a wall seen as little as twice (two
`map_update_step` nudges from 0.5). Now, with this on, it blocks only on **locked
(confirmed)** walls; an unlocked `>wall` cell is passable at a penalty (cost 5).
A wall must therefore be *confirmed* (≥0.9, ~4 consistent observations) before it
can seal a route, so a transient phantom from a single drift episode — which often
never gets consistently re-confirmed at the same believed cell, so never locks —
no longer seals the agent in. Real walls lock quickly and block as before; the
true-map collision check still stops the agent at any real wall it routes through.

| config | completion | loc mean | loc max | warp | steps | recoveries | reopens |
|---|---|---|---|---|---|---|---|
| baseline | 14/30 | 1.327 | 21.11 | 25.60 | 2797 | 3271 | 168.6k |
| **nav_locked_only** | **17/30** | **1.038** | 21.11 | 20.96 | 2396 | **1798** | **86.7k** |

+3 completions, recoveries and reopens roughly halved, mean error and steps down.
It notably fixes **bsp1011** — the pure-navigation livelock the diagnostics had
flagged (localized to ~0.16 cell yet 397 recoveries): the seal was unlocked walls,
so confirming-before-blocking dissolves it. **World mode is neutral** (30/30,
0.242→0.241, warp 0.02 either way), so it is safe to default on.

**(b) Merge conflict reconsideration (`merge_reconsider`, default OFF).** Idea:
when two agents' maps both lock a cell but disagree, distrust both and re-sense.
Measured **net-negative** in every variant:

| variant | completion | loc max |
|---|---|---|
| symmetric (any disagreement) alone | 16/30 | 12.39 |
| symmetric + nav_locked_only | 16/30 | 11.91 |
| asymmetric (only clear my-wall vs peer-free) alone | 15/30 | 19.84 |
| asymmetric + nav_locked_only | 15/30 | 19.84 |

It also *drags `nav_locked_only` down* (17→16). Root cause is the **local-mode
frame mismatch**: agents in different believed frames disagree on the same index
*legitimately*, so reconsidering removes correct geometry more often than it clears
a phantom (the asymmetric "clear contested walls" version is worse still — it opens
real walls a frame-shifted peer reports free, causing new bsp seal failures). The
one upside is lower worst-case drift (max 21→12 for the symmetric version). Kept as
an opt-in lever; not shipped. The proper fix for cross-frame merging is map
registration / loop closure, still the outstanding back-end gap.

## 22. Occlusion gating — don't sense through confirmed walls (biggest local-mode win)

Symptom: in `local` mode, walls were occasionally written on *the far side* of a
known wall. The cell FOV (`sensor_model.sense`) is recursive-shadowcast against the
**true** map, so it never sees through a real wall — but `local` mode re-anchors
every observation to the **believed** pose (`believed_cell - true_cell` shift).
Under drift that offset slides a genuine reading *behind* a believed locked wall,
so the integration corrupts a cell the agent has no line of sight to. Compounded
over a run this is the dominant source of the catastrophic-drift failures.

Fix (`base_agent._filter_occluded`, `occlusion_block`, default ON): before
integrating, drop any observation whose Bresenham ray *from the believed pose*
crosses a **locked** wall. Only locked (confirmed) walls occlude — a still-
accumulating wall isn't trusted to hide things — and the target cell is never its
own occluder, so the wall being looked at is still mapped; only cells beyond it are
blocked.

This is the **single largest improvement** of the whole effort, on the all-on hard
set (30 runs, local mode):

| occlusion_block | completion | loc mean | loc max | warp | steps |
|---|---|---|---|---|---|
| off | 17/30 | 1.038 | 21.11 | 20.96 | 2396 |
| **on** | **27/30** | **0.364** | **2.47** | **3.16** | **992** |

+10 completions; worst-case drift **21.1 → 2.47 cells**; warp 21 → 3.2; and it is
*faster overall* (runs finish instead of thrashing to max_steps: 90k agent-steps
vs 216k). **World mode is neutral** (30/30, 0.248, warp 0.01) — there is no drift
offset, so the believed-frame ray check essentially never fires. The Bresenham per
observation adds ~0.2–0.5 ms/agent-step, far outweighed by the shorter runs.
Remaining hard-set failures: room1089, room1071, cave1137.

Generalization: a 30-run batch of **fresh random seeds** (the new overfit check,
see `docs/TESTING.md`) scores **26/30** with this config — on par with the curated
set, i.e. the gains are not overfit to the curated seeds.

## Updated tunables (`config.py`, additions)
```
swarm_slam              True   inter-agent pose anchoring
slam_anchor_sigma       1.0    spread of an inter-agent relative measurement (cells)
slam_jump_margin        2.0    teleport-rejection slack beyond |motion| (cells)
slam_max_endpoints      24     scan endpoints scored per particle (speed knob)
map_anchor              "world" | "local"   ground-truth vs robot-anchored map
slam_trusted_weight     0.0    EXPERIMENTAL extra weight on trusted-wall hits (local; off)
lock_erosion            True   unlock a cell repeatedly contradicting its locked state
lock_erosion_patience   5      consecutive contradictions before unlocking
search_recovery         True   survey furthest frontier when stalled against a seal
search_window           30     steps of believed-pose history for the stall detector
search_min_progress     1.5    net cells below which (with block frac) = stalled
search_block_frac       0.5    min blocked-step fraction in the window for a real seal
search_linger           6      steps to dwell at a survey target so its wall can erode
erosion_protect_steps   30     steps a freshly-eroded cell resists a peer re-locking it
nav_locked_only         True   A* blocks only on LOCKED walls (unlocked >wall = penalised)
merge_reconsider        False  EXPERIMENTAL: re-sense a wall a peer contradicts (net-negative)
occlusion_block         True   drop observations whose ray crosses a locked wall (local mode)
```

## Net state after Part II
- `world` mode (benchmark): ~0.37 cell error, 60/60 completion, map == truth.
- `local` mode (real SLAM): with `sensor_range 8` + jump-gate + lock erosion +
  search mode + targeted erosion protection, the believed-frame map self-heals
  displaced walls and the agent surveys its way out of a seal instead of jittering.
  On the plain-selected A/B set: **27/30, mean ~0.39, worst-case ~5.3 cells.**
- A new **all-features-on hard set** (`tools/sweep.py` STANDARD_SEEDS, selected to
  surface remaining weaknesses) started at **14/30**, reached **17/30** with
  `nav_locked_only` (§21), and is now at **27/30** with occlusion gating (§22) —
  the single biggest win, which collapsed the catastrophic-drift cases (worst-case
  error 21→2.5 cells). A fresh-random generalization batch scores **26/30** with
  the same config, so the gains are not overfit to the curated seeds. Remaining
  failures: room1089, room1071, cave1137.
- The remaining `local` gap (and the residual maze aliasing from Part I) is still
  the same fundamental limit for the drift cases: **no loop closure / pose-graph
  back-end** — re-anchoring against the known start region on the return trip is
  the natural next step. The livelock/thrash cases are instead a recovery-logic
  problem, not a SLAM one.
