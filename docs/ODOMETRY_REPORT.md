# Odometry Localization Report

A short companion to `SLAM_REPORT.md`, covering the pure odometry-based
localizer — the baseline the SLAM work was measured against.

---

## The method

`localization/odometry.py` is **dead reckoning**: each step it integrates the
robot's motion and adds Gaussian drift, using no sensor information at all.

```
believed_position += actual_motion + N(0, odometry_noise)   # per axis, per step
```

That's the whole algorithm. `actual_motion` is the true per-step displacement;
`odometry_noise` (config) models wheel slip / encoder error. Observations and
the map are accepted in the signature but ignored — there is no correction step.

A useful reference point: `localization/exact.py` is the **same integration with
zero noise**, which (because the sim feeds exact motion) tracks the true pose
perfectly. So odometry *is* `exact` plus injected drift — the noise is the only
error source.

---

## Why it behaves the way it does

With no correction, the position error is a **random walk**: it grows without
bound, roughly as `odometry_noise · √(steps)`. There is nothing pulling it back,
so once it drifts it stays drifted and keeps drifting.

This produced three observable failures (all visible in testing):

1. **Unbounded divergence.** Under 0.1–0.5 drift the belief wandered 10–25 cells
   from the truth over a run.
2. **Beliefs inside walls.** Because nothing constrains the estimate to free
   space, the believed cell sat inside a wall **60–90 %** of the time once
   drifted — a physically impossible state the localizer had no way to notice.
3. **Cascade into navigation.** Planning and exploration run on the belief, so a
   wrong belief means planning from the wrong cell, following un-followable
   paths, and rarely reaching the goal. At the extreme, the belief drifted
   *off the grid* entirely, which originally crashed the A\* planner.

---

## What was actually "fixed"

There is little to fix *within* pure odometry — drift is inherent to dead
reckoning, and the proper fix is to add a correction step (i.e. SLAM, see the
other report). The odometry path itself only received:

- **Out-of-bounds guard in A\*** — when a divergent belief put the planning
  start/goal off the map, the planner now returns no path instead of crashing on
  an out-of-range index. (Also benefits SLAM, but odometry is what exposed it.)
- **Signature alignment** — `update()` accepts `observations` / `internal_map`
  for a uniform localizer interface, even though odometry ignores them.

Everything else — keeping the estimate in free space, bounding the drift,
recovering from divergence — is *exactly* what odometry cannot do and what
motivated the move to a particle filter.

---

## Role now

Pure odometry is retained deliberately as the **divergent baseline** for
comparison (`"localization": "odometry"`), alongside `exact` (the perfect
baseline). The contrast is the headline result of the SLAM work:

| localizer | typical error (0.1–0.5 drift + noise) | belief in walls | reaches goal |
|-----------|---------------------------------------|-----------------|--------------|
| `exact`   | ~0.2 cell (one-step integration lag)  | 0 %             | yes |
| `odometry`| 10–25 cells, growing                  | 60–90 %         | rarely |
| `slam`    | ~0.2–0.5 cell, bounded                | 0 %             | yes |

In short: odometry is correct only instantaneously and decays from there. It's
valuable precisely as the "what happens with no correction" control case that
shows why the SLAM correction is needed.

### Tunable (`config.py`)
```
localization     "odometry"
odometry_noise   0.1   per-step drift std-dev (0 → equivalent to `exact`)
```
