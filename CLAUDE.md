# Repo guidance for Claude

A Python 2D grid-based robotics exploration simulator (occupancy mapping +
Monte-Carlo-localization SLAM + frontier exploration + optional multi-agent
swarm). Entry point `main.py`; all tunables in `config.py`.

## Keep the design reports up to date (important)

When you make a change that affects **localization / SLAM / occupancy mapping**
(anything under `localization/`, the mapping/locking logic in
`agents/base_agent.py`, the sensor model, or the SLAM-relevant parts of the
multi-agent / communication code), you MUST also update **`docs/SLAM_REPORT.md`**
in the same change. For changes to the pure dead-reckoning baseline
(`localization/odometry.py`, `localization/exact.py`), update
**`docs/ODOMETRY_REPORT.md`** instead. If a change clearly belongs to neither but
warrants a writeup, create/extend the most relevant report under `docs/`. Changes
to the **test infrastructure itself** (the sweep harness, the standard seed set,
throughput) and a running log of sweep results go in **`docs/TESTING.md`**.

What to record (append in chronological order, matching the existing style):

- **The change and its motivation** — what problem/symptom prompted it.
- **The empirical results** — the actual measured numbers from the runs you did
  (completion rates, localization error mean/max, map-warp, ms/step, etc.). Use
  real measurements; never invent or estimate numbers.
- **Failures as well as successes** — if you tried variants that didn't work,
  record them with their numbers and why they were rejected/reverted. The
  negative results are as valuable as the positive ones.
- **Be detailed about the process, but add no filler.** No marketing language, no
  speculation presented as fact, nothing fabricated. If something is uncertain or
  untested, say so.

Note the configuration the numbers were measured under (e.g. `sensor_range`,
`num_agents`, `map_anchor`) when it affects the result, since defaults change
over time.

## Running and measuring (headless)

The project uses a local venv. Run a single headless (no GUI) run with:

```
cd /home/devon/Repo/Robotics-Sensing
PYTHONPATH=. MPLBACKEND=Agg .venv/bin/python main.py
```

For empirical testing use the parallel sweep harness (the standard way to get
comparable numbers; it runs runs in parallel across cores and reports completion,
localization error, map-warp, avg steps, and ms/step):

```
PYTHONPATH=. MPLBACKEND=Agg .venv/bin/python tools/sweep.py                 # standard set
PYTHONPATH=. MPLBACKEND=Agg .venv/bin/python tools/sweep.py --overrides '{...}'   # A/B a change
```

The **standard set** is the curated 30 hard seeds (`STANDARD_SEEDS` in
`tools/sweep.py`): 3 agents × maps {room, bsp, cave} × 10 selected seeds, at
41×41. Use it for both sides of an A/B so numbers are comparable; see
`docs/TESTING.md` for the set, the selection methodology (and the overfitting
lesson — don't tune against a fixed set until it stops discriminating), and how to
refresh it with `--select`. Determinism: set `map_seed` / `behaviour_seed`.
Earlier report tables predate this set and used seeds 1–20; note which set a
number came from.

## Orientation

- `config.py` — every tunable, with inline comments. Read it first.
- `localization/` — `slam.py` (particle filter), `odometry.py`, `exact.py`.
- `agents/base_agent.py` — per-step loop: sense → map → localize → plan → move →
  communicate; occupancy update + cell locking + lock erosion; recovery (hard
  recovery + search/survey mode for escaping sealing phantom walls).
- `tools/sweep.py` — parallel empirical sweep + seed selection + diagnostics; see
  `docs/TESTING.md`.
- `core/` — `simulation.py` (loop, termination), `agent.py` (factory),
  `environment.py`, `map.py`.
- `communication/` — `communication_model.py` (map merge), `swarm_coordinator.py`.
- `exploration/frontier.py`, `planning/astar.py`, `sensing/`, `rendering/`,
  `maps/` (generators).

When unsure whether the SLAM behaviour changed, verify with a seeded headless
sweep before and after, and compare — don't assume.
