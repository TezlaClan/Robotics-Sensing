# Testing & empirical sweeps

This doc owns the **test infrastructure and a chronological log of test results**.
The mechanics of how the sweep is run, which seeds are standard, and how fast it
goes live here. Findings about *localization/SLAM/mapping behaviour* still belong
in `SLAM_REPORT.md`; findings about the dead-reckoning baseline in
`ODOMETRY_REPORT.md`. This file is the place for: changes to the harness itself,
the seed set, timing/throughput, and a running table of sweep numbers so results
stay comparable over time.

## The harness: `tools/sweep.py`

Headless, no rendering, parallel across independent runs.

```
# Standard regression sweep (curated hard seeds, all cores):
PYTHONPATH=. MPLBACKEND=Agg .venv/bin/python tools/sweep.py

# A/B a config change:
PYTHONPATH=. MPLBACKEND=Agg .venv/bin/python tools/sweep.py \
    --overrides '{"search_recovery": false}'

# Difficulty scan over a fresh pool (seed selection):
PYTHONPATH=. MPLBACKEND=Agg .venv/bin/python tools/sweep.py \
    --seeds 1000-1059 --select 60
```

Flags: `--maps`, `--seeds` (`a-b` or `a,b,c`; default = curated set),
`--overrides` (JSON config patch), `--agents`, `--map-size`, `--max-steps`,
`-j/--jobs` (default = all cores), `--per-run`, `--select N`, `--track-paths DIR`,
`--random N` / `--random-seed S`, `--set {easy,hard,impossible}`.

### Generalization batch (overfit check)

`--random N` (default **10**) runs, *in addition to* the curated set and reported
as a separate section, **N fresh random seeds per map** drawn from a range disjoint
from every curated/historical seed. The picker is entropy-seeded, so it sees
**different maps every run** — this is a deliberate anti-overfit check, *not* a
precise benchmark (its numbers vary run to run). If the curated set looks great but
this batch is much worse, the solution is overfit to the curated seeds. The seeds
it used are printed so a surprising result can be reproduced (or pass
`--random-seed S`). Use `--random 0` to skip it for a fast curated-only A/B.

Reported metrics: completion (overall + per map), localization error mean/max,
map-warp (decided cells disagreeing with ground truth, per agent), avg steps,
**compute ms/agent-step** (per-run CPU, comparable regardless of `-j`), and total
**wall time**.

### Oracle distance & progress (`opt_dist`, `prog`)

`--per-run` now also prints `opt=<N>` and `prog=<0..1>`. `opt_dist` is the **true
shortest-path distance** start→goal, computed by an A* oracle over the *full* map
(`planning/astar.true_path_distance`, 4-connected free-space cells — matching what
the agent's planner navigates), **not** straight-line. The two diverge enormously
on mazes: e.g. maze2128 is 16.3 cells straight-line but **277 cells** along the only
free path (17×); maze2391 39.8 → 314 (7.9×); open room2167 only 1.3×. This is why a
maze run that "completes fine" still takes thousands of steps — the path really is
that long. The oracle is measurement-only; the agents never see it.

`prog` (mission progress, 0→1, used to grade non-completing runs by closeness) is
now measured against this oracle path distance instead of straight-line, for both
the start→goal→start total (`2·opt_dist`) and the claimer's *remaining* distance
(an oracle path from its true cell to the target). Before, a maze failure could
show a misleadingly high progress because the goal was straight-line-near while
path-far. (Cached `progress` values in the pre-2026-06-19 search JSONs predate this
and used straight-line; re-run if exact comparability is needed.)

### Diagnostics ("how often did the agent think the map was wrong")

Every run also reports recovery/event counters (per-run with `--per-run`, summed
in the `events:` aggregate line). These quantify how much the agent fought its own
map — a run that completes only after thousands of reopens is fragile even though
it "passed":

- `recoveries` — hard recoveries (unlock-all + reopen boundary + step), i.e. fully
  frozen for `stuck_limit` steps.
- `searches` — times **search mode** was entered (stalled against a seal).
- `erosions` — locked cells eroded back to unknown (lock erosion firing).
- `reopens` — believed-wall cells reopened by boundary recovery.
- `jump-gates` — SLAM teleport rejections (perceptual-aliasing jumps gated).

**Full path tracking** (`--track-paths DIR`): writes `DIR/<map>_<seed>.json` per
run with every agent's per-step `[step, true_x, true_y, bel_x, bel_y, err,
search_flag]`. Off by default (one file ≈ 0.5 MB for a 4000-step run); use it to
replay/plot a specific failing seed.

## Curated seed sets

Three sets in `tools/sweep.py`, all at `map_width/height = 41`, `num_agents = 3`,
cut from searches over seeds 2000–23xx with `behaviour_seed = map_seed`, all
recovery features ON (room/bsp/cave: a 999-run search; maze: a separate 400-run
search). Select with `--set {easy,hard,impossible}` (default `hard`). With a set,
`--maps` defaults to that set's own map keys. Replay any seed by setting BOTH
`map_seed` and `behaviour_seed` to it.

- **`EASY_SEEDS` (24):** the fastest completers per map (room/bsp/cave/maze, 6
  each), all completing in **< 400 steps** (slowest member 66). A quick
  ~100%-completion smoke/regression check (**24/24, avg 25 steps, ~0.6 s**) to run
  *before* the longer sets.
- **`HARD_SEEDS` (40):** slowest completers (balanced per map) + failures spread
  across "closeness to finishing" (75% returning-home … 0% never-reached-goal),
  ~half/half. Discriminating both ways — failures can be rescued by a fix, slow
  completers can regress. Baseline **20/40**. Includes maze (5 slow completers +
  5 spread failures).
- **`IMPOSSIBLE_SEEDS` (60):** seeds that failed at `behaviour = map` — room 12,
  bsp 1, cave 17 (3% of the 999-run search) + a 30-seed closeness-spread of maze
  failures (maze failed 240/400; too many to list all, so the persisted slice is
  representative — full set reproducible from the searches). Baseline **0/60** by
  construction. Not literally unsolvable (most are behaviour-specific), but the
  genuinely hard cases — use to hunt failure modes / confirm a fix rescues them.

**Search findings (2026-06-18):**
- room/bsp/cave: 30/999 fail (3%) — concentrated in **cave** (17) and **room**
  (12); **bsp** robust (1). Closest failures reach the goal and stall on the way
  home (~75%); the rest (mostly cave) never reach it (localization-drift wall).
- **maze: 160/400 complete — 60% FAIL**, by far the hardest. Maze is all narrow
  corridors, where a local-mode belief offset *along* a corridor is unobservable
  (the same aperture problem behind the goal-claim deadlock); room/bsp/cave have
  open areas and corners that pin the pose. Maze is the clearest case for a
  loop-closure back-end. It is curated into the sets but kept off the default
  `--maps` for the older room/bsp/cave numbers' comparability.

Re-cut with `/tmp/search1000.py` / `/tmp/search_maze.py` after a difficulty shift.

### Earlier set — the overfitting lesson (kept for context)

The original standard set was maps {room,bsp,cave} × seeds **1–20** (60 runs).
Reusing the same 20 seeds for every experiment overfit the tuning to them: by
mid-development almost every run completed (56/60) regardless of the change under
test, so the sweep had **lost discriminating power** — it could no longer tell a
real improvement from noise. Concretely, search mode measured **56/60 with and
without** it on that set, hiding a genuine effect (see results log).

### Why these seeds (and the overfitting lesson)

The original standard set was maps {room,bsp,cave} × seeds **1–20** (60 runs).
Reusing the same 20 seeds for every experiment overfit the tuning to them: by
mid-development almost every run completed (56/60) regardless of the change under
test, so the sweep had **lost discriminating power** — it could no longer tell a
real improvement from noise. Concretely, search mode measured **56/60 with and
without** it on that set, hiding a genuine effect (see results log).

The current set was selected by scanning a fresh pool with `--select` and is kept
deliberately **hard**:

1. **Fresh pool.** Difficulty was scanned over **1000–1149** (150 per map),
   disjoint from the old 1–20.
2. **Selected with ALL recovery features ON** (the shipped default). These are the
   seeds that are *still hard with everything enabled* — the system's remaining
   weaknesses — so the set tracks real failure modes and verifies fixes against
   them. Failure counts in the pool: **room 7, bsp 1, cave 16** (cave is the hard
   map). Per-map difficulty profile: **cave** is the localization stressor (drift
   failures, max error to ~10–21 cells), **room** is mixed (some catastrophic
   drift seeds + recovery-thrash seeds), **bsp** is navigation-bound (localization
   easy, ~0.2 err; difficulty is path length / seals).
3. **Mostly-hard + guards.** Each map's 10 seeds are the genuine failures +
   hardest-solvable, plus ~2 easy guards (which complete cleanly) so a *regression*
   on an easy case is still caught. All-features-on baseline completion is
   **14/30**, with plenty of headroom in both directions.

**Selection bias — important.** Because the set is selected under the full default
config, it is enriched for that config's failures, so it is *not* neutral for
feature-vs-feature A/B. We hit this concretely earlier: selecting under the default
(erosion on) made `lock_erosion` look net-negative (26/30 off vs 21/30 on), but
re-selecting under the plain config (both recovery features off) showed it clearly
net-positive. **For an unbiased A/B, re-select over the pool with the feature(s)
off** (e.g. `--overrides '{"search_recovery": false, "lock_erosion": false}'
--select N`); for tracking the current system's weaknesses, the default all-on set
is what you want. The previous plain-selected set (room `[1055,1002,1029,1017,
1053,1040,1011,1018,1025,1021]`, bsp `[1045,1057,1035,1059,1044,1050,1026,1032,
1014,1037]`, cave `[1013,1058,1055,1027,1037,1039,1044,1018,1009,1052]`) is kept
here for that purpose.

**Caveat:** "hardest" is relative to the code at selection time. After a change
that substantially alters localization/mapping, re-run `--select` and refresh
`STANDARD_SEEDS` (record it here).

## Speed

Goal was to cut sweep wall-time without losing accuracy. What was tried:

| Lever | Effect | Adopted? |
|---|---|---|
| **Multiprocessing** across runs (`-j`) | 5–10× wall-time; **zero** accuracy/behaviour change (runs are independent) | **Yes** (default = all cores) |
| **Fewer, harder runs** (60→30) | Halves run count *and raises* discrimination per run | **Yes** |
| **Smaller map** | Does **not** cut per-step cost; only shortens runs by making them easier | **No** |

Notes / measurements (12-core box):

- **Threads vs processes.** The per-step sim loop is GIL-bound pure Python (numpy
  releases the GIL only inside the vectorized SLAM weight step), so threads barely
  help. Independent runs are embarrassingly parallel, so separate **processes**
  scale near-linearly. On the old 60-run set: ~292 s serial CPU → **52.6 s wall at
  `-j 12` (≈5.5×)**. Sub-linear because the handful of 4000-step *failure* runs
  dominate the tail and `Pool.map` waits on the slowest.
- **Fewer runs.** 30 curated hard runs replace 60 fixed ones. Default sweep wall
  time is **~27–46 s at `-j 12`** depending on config (failure-heavy configs run
  longer). End-to-end this is faster than the old 60-run serial workflow *and* the
  set discriminates better.
- **Smaller map — evaluated, rejected.** ms/agent-step is essentially flat with
  map size (41→2.11, 31→2.21, 25→2.26 ms): the 200-particle filter dominates and
  the map-derived distance field is cached/vectorized, so per-step cost does not
  scale with area. A smaller map only reduces *steps per run* by making maps
  easier — which sacrifices the difficulty we deliberately selected for and breaks
  comparability with every number in `SLAM_REPORT.md` (all measured at 41×41). Not
  worth it given parallelism already gives the win for free.

Current baseline compute cost: **~2.1–2.7 ms/agent-step** (3 agents, sensor_range
8, 200 particles, 24 endpoints), config-dependent.

---

## Results log

Append newest at the bottom. Record the config delta, the set used, and real
measured numbers.

### 2026-06-17 — New harness + standard set established

Migrated the ad-hoc `/tmp` sweep into `tools/sweep.py` (parallel + timing),
selected the new standard set (above), and ran the **2×2 recovery-feature matrix**
on it. Standard set (30 runs, local map mode, 3 agents, `-j 12`):

| config | completion | loc mean | loc max | warp | avg steps |
|---|---|---|---|---|---|
| vanilla (both off) | 21/30 | 0.843 | 15.33 | 13.87 | 1803 |
| + lock_erosion | 26/30 | 0.628 | 26.67 | 9.40 | 1159 |
| + search_recovery | 21/30 | 0.557 | 13.67 | 13.78 | 1968 |
| + both (shipped default) | **27/30** | **0.447** | **12.09** | **8.80** | **1010** |

Reads:

- **lock_erosion** is clearly net-positive on the fair set: +5 completion, warp
  13.9→9.4, mean err 0.84→0.63. (Its one wart: a single cave seed's *max* drift
  rises to 26.7 — erosion can briefly worsen one excursion before re-healing.)
  This corrects the biased earlier reading; see the selection caveat above.
- **search_recovery alone** is flat on completion (21→21) and even *hurts* cave
  (6→4): on an open, drift-prone map it sends the agent surveying and it times
  out. But it lowers mean and max error.
- **Interaction (the real result):** search and erosion *synergise*. Search re-
  surveys the region so a sealing phantom wall is re-observed; erosion is what
  actually clears it. Without erosion the re-survey is fruitless (cave 4/10); with
  erosion it pays off — adding search on top of erosion takes cave 7→8, **max
  drift 26.7→12.1**, mean 0.63→0.45, warp 9.4→8.8, steps 1159→1010. Shipping both
  on (the config default) is the best cell on every metric.

Old 1–20 set for contrast (60 runs): search on vs off was **56/60 either way** —
the effect above was entirely invisible on the overfit set. (This 2×2 was on the
plain-selected set, now retained as the "unbiased A/B" set above.)

### 2026-06-17 — Search refinements, erosion protection, harder set

Added per-run diagnostics + path tracking, three behaviour changes, and re-cut the
standard set harder (see SLAM_REPORT §20 for mechanism detail).

- **Search linger + give-up.** Dwell at a survey target so its wall can erode;
  mark lingered-without-opening targets visited and give up once exhausted. A bug
  where the dwell counted as "stuck" regressed the (then plain-selected) set to
  23/30 with ~100k boundary reopens until lingering was excluded from the stuck/
  blocked detectors.
- **Multi-agent erosion protection.** Diagnostics caught a blanket version making
  it *worse* (23/30, recoveries 410→956, reopens 33k→100k): it shielded *bad*
  erosions of real walls from peer correction. Targeted version (protect only an
  eroded cell we *still* read as free) is the best result of the whole effort on
  the plain set: **27/30, loc 0.388, max 5.32 (was 12.09), warp 7.17, jump-gates
  293→30.**
- **New hard standard set** (above): scanned 1000–1149 (450 runs) all-features-on,
  re-cut to 10/map weighted toward genuine failures. All-on baseline:

  | | completion | loc mean | loc max | warp | steps | recoveries | reopens | gates |
  |---|---|---|---|---|---|---|---|---|
  | new set, all-on | **14/30** | 1.327 | 21.11 | 25.60 | 2797 | 3271 | 168.6k | 535 |

  Per-map: room 3/10, bsp 9/10, cave 2/10. The diagnostics separate failure modes,
  which is the point of this set:
  - **Catastrophic-drift / recovery-thrash** (SLAM failure): e.g. cave1116 (err
    4.75/21.11, 147 rec, 228 gates), room1031 (6.68/16.41, 253 rec, 151 gates),
    cave1137 (3.25/10.17, 303 rec). The agent fights its map for the whole run.
  - **Clean stalls** (not thrashing): cave1055 (err 1.36, 0 rec), cave1058 (1.25,
    1 rec), room1089 (1.17, 0 rec) — stuck/slow without recovery churn.
  - **Pure navigation livelock** (localization fine!): **bsp1011** fails with err
    only **0.16/0.68** but 397 recoveries / 365 searches / 39.7k reopens — perfectly
    localized yet unable to get unstuck. The clearest "fix the recovery logic, not
    the SLAM" target in the set.

### 2026-06-17 — Locked-only navigation + merge reconsideration

Two proposed upgrades, A/B'd on the all-on hard set (baseline 14/30):

| config | completion | loc mean | loc max | warp | steps | recoveries | reopens |
|---|---|---|---|---|---|---|---|
| baseline | 14/30 | 1.327 | 21.11 | 25.60 | 2797 | 3271 | 168.6k |
| nav_locked_only (A) | **17/30** | 1.038 | 21.11 | 20.96 | 2396 | 1798 | 86.7k |
| merge_reconsider (B) | 16/30 | 0.921 | 12.39 | 20.36 | 2529 | 2696 | 185.7k |
| A + B | 16/30 | 0.995 | 11.91 | 19.98 | 2421 | 2167 | 168.5k |
| A + B(asymmetric) | 15/30 | 1.158 | 19.84 | 21.86 | 2502 | 1577 | 116.2k |

- **A (locked-only navigation): shipped ON.** +3 completion, recoveries/reopens
  ~halved; fixes the bsp1011 navigation livelock; **world mode neutral** (30/30,
  0.242→0.241). A wall must be *confirmed* (locked) before it blocks a path, so
  transient phantoms can't seal the agent in.
- **B (merge_reconsider): not shipped (default off).** Net-negative in every
  variant and drags A down (17→16); the asymmetric "only clear contested walls"
  version is worse (opens real walls a frame-shifted peer reports free). Root cause
  is the local-mode frame mismatch — index-merging disagreements are mostly
  legitimate frame differences. It does cut worst-case drift (max 21→12), so it is
  kept as an opt-in lever. Real fix needs map registration / loop closure.

### 2026-06-17 — Occlusion gating + generalization batch

- **Occlusion gating (`occlusion_block`, shipped ON)** — the biggest single win.
  Drop any observation whose ray from the believed pose crosses a *locked* wall, so
  drift can't write through confirmed walls in local mode (SLAM_REPORT §22). On the
  hard set: **17→27/30**, loc mean 1.04→0.36, **max 21.1→2.47**, warp 21→3.2, and
  *faster* (runs finish: 90k vs 216k agent-steps). World mode neutral (30/30, 0.248).
  Remaining hard-set failures: room1089, room1071, cave1137.
- **Generalization batch added** (`--random`, default 10/map). With the full
  default config: curated **27/30** vs random **26/30** (loc 0.42, max 8.46) — the
  random, never-tuned maps perform on par with the curated set, so the cumulative
  gains generalize rather than overfitting the curated seeds.

### 2026-06-17 — Mapping speed (compute + exploration)

Profile-led; see SLAM_REPORT §23–24. cProfile of the mapping path (after the
filter was already optimized) showed `_merge_maps` was **not** a hotspot (1%); the
costs were `range_scan` (21%), `sense` FOV (15%), and a triple-wrapper
`is_free`/`in_bounds` chain.

- **Compute, shipped (content-preserving):** fast inline grid lookups in the ray
  casters + a per-cell FOV shadowcast cache. Profile total **7.49→5.95 s
  (−20.6%)**, `range_scan` −53%, `sense` −64%, **bit-identical** (world 30/30 and
  hard 27/30 reproduce exactly). Per-step cost is now dominated by the (untouched,
  accuracy-critical) SLAM filter.
- **Exploration efficiency, rejected:** gain-weighted frontier selection
  (`frontier_gain_weight`) — best case ~3% fewer steps but regresses accuracy
  (max drift 2.47→3.59, warp 3.16→4.83); default 0. Note: hard-set avg-steps is
  dominated by the 3 capped non-completers (~400 of 992), so the real step lever is
  fixing those drift failures (loop closure), not frontier selection.

### 2026-06-18 — Goal-claim deadlock fixed; seed methodology flaw exposed

Running room1089 directly (a user catch) revealed the curated set conflates **map
difficulty with behaviour luck**: each seed is a single `(map_seed, behaviour_seed
= map_seed)` sample. room1089's goal is ~6 cells from start (trivial map), yet
`behaviour_seed=1089` deadlocks — and ~1/3 of behaviour seeds fail it. So "hard
seed" ≠ "hard map".

- **Deadlock fix (SLAM_REPORT §25):** the 3 standing failures were all a moving
  limit-cycle (claimer oscillates in place under a ~1-tile belief offset; the
  blocked-fraction stall gate missed it because the agent keeps moving). Added a
  long-horizon no-progress detector + a target-directed true-space recovery walk.
  **Hard set 27→30/30** (avg steps 992→692), **generalization 45/45**, world mode
  preserved (30/30, 0.247).
- **Consequence — set is now saturated (30/30), so it no longer discriminates.**
  TODO (next): re-cut `STANDARD_SEEDS` using a **robust difficulty measure that
  averages each map over several behaviour seeds**, so a seed is "hard" only if the
  *map* is consistently hard, not a one-off behaviour fluke. This both re-grounds
  the set and gives it teeth again.

### 2026-06-18 — 999-run search; HARD and IMPOSSIBLE sets cut

Ran maps {room,bsp,cave} × seeds 2000–2332 (999 runs, `behaviour=map`, all features
on). **969/999 complete; 30 fail (3%)** — room 12, bsp 1, cave 17. Two sets cut and
persisted in `tools/sweep.py` (`--set hard|impossible`); baselines: HARD **15/30**,
IMPOSSIBLE **0/30**. All 30 failures, by closeness to finishing:

```
returning (reached goal, stuck on the way home):
  room2302 75%  cave2190 74%  cave2081 71%  cave2119 70%  room2274 68%
  bsp2205 66%   cave2062 61%  cave2183 49%  room2223 46%  cave2328 43%
to-goal (never reached goal):
  cave2038 40%  cave2134 34%  room2262 10%  room2069 7%
  0%: room2018 room2149 room2163 room2167 room2230 room2299 room2304
      cave2001 cave2010 cave2019 cave2154 cave2155 cave2175 cave2251 cave2263 cave2285
```

Pattern: the "close" failures (reach goal, fail returning) span all three maps; the
"never reach goal" failures are mostly **cave** (drift in open space) plus several
**room** seeds with low error (0.3–1.3) — likely a residual navigation/seal issue,
not drift. These are the next things to chase. (Caveat: `behaviour=map` is one
sample; some of these complete under other behaviour seeds — the same conflation
noted above. A multi-behaviour re-cut remains the robust follow-up.)

### 2026-06-18 — maze search + EASY set; sets now cover 4 map types

Separate **400-run maze search** (seeds 2000–2399, behaviour=map, all on):
**160/400 complete — 240 fail (60%)**. Maze is the hardest geometry by a wide
margin (vs ~3% on room/bsp/cave) — all-corridor maps make the along-corridor
belief offset unobservable. Folded into the sets "sensibly": HARD += 5 slow maze
completers + 5 spread maze failures (→40); IMPOSSIBLE += a 30-seed closeness-spread
of the 240 maze failures (→60, not all 240). Added **`EASY_SEEDS`** (24): fastest
completers/map across all 4 types, all < 400 steps — **24/24, avg 25 steps, 0.6 s**
as a pre-flight regression check. `--maps` now defaults to the selected set's map
keys so maze actually runs.

### 2026-06-19 — IMPOSSIBLE seeds × 20 behaviours; oracle distance metric

**Behaviour-spread test** of 5 IMPOSSIBLE seeds, each run across behaviour seeds
1–20 (map_seed fixed). Confirms the "impossible" label is a *mix* of behaviour
flukes and truly hard maps: room2167 **20/20**, bsp2205 **19/20** (easy maps that
failed only on the single behaviour=map sample); cave2019 **16/20** (genuinely
fragile — loc err_max 15.97 on some behaviours, the open-area drift gap);
maze2128 **1/20**, maze2391 **0/20** (genuinely map-hard, corridor aperture). Direct
evidence for the behaviour/map conflation flagged above. (Script + per-run JSON in
`/tmp`; every run reproducible via (map_type, map_seed, behaviour_seed).)

**Oracle distance.** Watching maze2128 showed agents completing but via an
*enormously* long path. Added `planning/astar.true_path_distance` — an A* over the
full map giving the true shortest free-space path start→goal, and wired it into the
harness as `opt_dist` and into the `progress` metric (replacing straight-line; see
the metrics section). maze2128 is 16.3 cells straight-line but **277 along the path
(17×)**, maze2391 39.8→314, room2167 34.5→45. So the long maze runs aren't the
agent wandering — the optimal path itself is hundreds of cells, and the SLAM/search
overhead is now measurable against it (e.g. maze2126 completes in 2517 steps for a
139-cell optimum). Measurement-only; agents don't use it.
