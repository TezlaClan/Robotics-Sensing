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
`-j/--jobs` (default = all cores), `--per-run`, `--select N`, `--track-paths DIR`.

Reported metrics: completion (overall + per map), localization error mean/max,
map-warp (decided cells disagreeing with ground truth, per agent), avg steps,
**compute ms/agent-step** (per-run CPU, comparable regardless of `-j`), and total
**wall time**.

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

## Standard seed set

Defined in `STANDARD_SEEDS` in `tools/sweep.py`: 10 seeds each for `room`, `bsp`,
`cave` (30 runs), at the default `map_width/height = 41`, `num_agents = 3`.

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
