"""
tools/sweep.py

Headless empirical sweep harness for the Robotics-Sensing simulator.

Runs the simulation across a set of (map x seed) cells with no rendering, and
reports completion, localization error (mean/max), map-warp, steps, and compute
time (ms/step). Independent runs are executed in PARALLEL across processes
(multiprocessing) - the per-step sim loop is GIL-bound pure Python, so threads
do not help, but separate processes scale near-linearly with cores. The timing
numbers are per-run CPU cost (so they are comparable regardless of -j), plus the
total wall time of the whole sweep.

Examples:
  # Standard regression sweep (the curated hard seed set), all cores:
  PYTHONPATH=. MPLBACKEND=Agg .venv/bin/python tools/sweep.py

  # A/B a config change (JSON overrides), explicit seeds, 8 workers:
  PYTHONPATH=. MPLBACKEND=Agg .venv/bin/python tools/sweep.py \
      --overrides '{"search_recovery": false}' -j 8

  # Difficulty scan over a fresh pool (for selecting a seed set):
  PYTHONPATH=. MPLBACKEND=Agg .venv/bin/python tools/sweep.py \
      --seeds 1000-1099 --per-run --maps room,bsp,cave

See docs/TESTING.md for the curated seed set and methodology.
"""
import argparse
import json
import math
import os
import sys
import time
from multiprocessing import Pool

from config import CONFIG
from utils.random_manager import RandomManager
from core.environment import Environment
from core.agent import create_agent
from communication.swarm_coordinator import SwarmCoordinator

from maps.room_generator import RoomGenerator
from maps.cave_generator import CaveGenerator
from maps.bsp_generator import BSPGenerator
from maps.maze_generator import MazeGenerator
from maps.mixed_generator import MixedGenerator
from maps.obstacle_generator import ObstacleGenerator

GENS = {
    "room": RoomGenerator,
    "cave": CaveGenerator,
    "bsp": BSPGenerator,
    "maze": MazeGenerator,
    "mixed": MixedGenerator,
    "obstacle": ObstacleGenerator,
}

# Curated standard regression set: a STRATIFIED hard set (failures + hard-but-
# solvable + an easy guard) per map, found by tools/sweep.py --select over the
# fresh pool 1000-1059 under the established baseline (see docs/TESTING.md). Kept
# in source so the standard sweep is reproducible. Map -> list of seeds.
# Curated hard set, selected by `--select` over pool 1000-1149 (150/map) with ALL
# recovery features ON (the shipped default). These are the seeds that are STILL
# hard with everything enabled - i.e. the system's remaining weaknesses - mostly
# genuine failures + hardest-solvable, plus a couple of easy guards per map for
# regression detection. Use it to track real failure modes and verify fixes.
# NOTE: because it was selected under the full default config it is biased toward
# that config's failures; for an unbiased feature-vs-feature A/B, re-select under
# the plain config (search+erosion off) - see docs/TESTING.md. Failure counts in
# the 150-seed pool: room 7, bsp 1, cave 16 (cave is the hard map).
STANDARD_SEEDS = {
    "room": [1031, 1034, 1089, 1071, 1093, 1074, 1055, 1040, 1014, 1092],
    "bsp":  [1011, 1098, 1010, 1111, 1063, 1103, 1128, 1027, 1144, 1083],
    "cave": [1116, 1087, 1000, 1137, 1067, 1019, 1055, 1058, 1124, 1064],
}


def run_one(task):
    """
    Run a single simulation cell. `task` is a dict so it pickles cleanly to a
    worker. Returns a metrics dict (no objects, so it pickles back cheaply).
    """
    map_type = task["map_type"]
    map_seed = task["map_seed"]
    behaviour_seed = task["behaviour_seed"]
    overrides = task["overrides"]
    map_size = task["map_size"]
    max_steps = task["max_steps"]
    track_dir = task.get("track_paths")

    cfg = dict(CONFIG)
    cfg.update(overrides)
    cfg["map_type"] = map_type
    cfg["map_seed"] = map_seed
    cfg["behaviour_seed"] = behaviour_seed
    cfg["render_live"] = False
    cfg["render_video"] = False
    cfg["debug"] = False
    if map_size is not None:
        cfg["map_width"] = map_size
        cfg["map_height"] = map_size
    if max_steps is not None:
        cfg["max_steps"] = max_steps

    rng = RandomManager(map_seed=map_seed, behaviour_seed=behaviour_seed)
    map_obj = GENS[map_type](cfg["map_width"], cfg["map_height"], rng).generate()
    env = Environment(map_obj)

    if cfg.get("map_sharing", "individual") == "shared":
        w, h = map_obj.width, map_obj.height
        sm = [[0.5 for _ in range(w)] for _ in range(h)]
        sl = [[False for _ in range(w)] for _ in range(h)]
        coord = SwarmCoordinator(shared_map=sm, shared_locked=sl)
    else:
        coord = SwarmCoordinator()

    start_pos = (map_obj.start[0] + 0.5, map_obj.start[1] + 0.5)
    n_agents = cfg.get("num_agents", 1)
    agents = [
        create_agent(i, start_pos, map_obj.width, map_obj.height, rng, cfg, coord)
        for i in range(n_agents)
    ]

    dt = cfg["dt"]
    sim_max = cfg["max_steps"]

    err_sum = 0.0
    err_n = 0
    err_max = 0.0
    steps = 0
    agent_steps = 0
    tracks = {a.id: [] for a in agents} if track_dir else None
    t0 = time.perf_counter()
    for step in range(sim_max):
        if coord.mission_complete:
            break
        for a in agents:
            if a.finished:
                continue
            a.step(env, agents, dt)
            agent_steps += 1
            e = math.hypot(a.believed_position[0] - a.true_position[0],
                           a.believed_position[1] - a.true_position[1])
            err_sum += e
            err_n += 1
            if e > err_max:
                err_max = e
            if tracks is not None:
                tracks[a.id].append([
                    step,
                    round(a.true_position[0], 3), round(a.true_position[1], 3),
                    round(a.believed_position[0], 3), round(a.believed_position[1], 3),
                    round(e, 3), int(a.search_mode),
                ])
        steps = step + 1
    elapsed = time.perf_counter() - t0

    # "Irrational map" / recovery diagnostics, summed across agents.
    n_recoveries = sum(getattr(a, "n_recoveries", 0) for a in agents)
    n_searches = sum(getattr(a, "n_search_entries", 0) for a in agents)
    n_erosions = sum(getattr(a, "n_erosions", 0) for a in agents)
    n_reopens = sum(getattr(a, "n_reopens", 0) for a in agents)
    n_gates = sum(getattr(a.localization_model, "_gate_total", 0) for a in agents)

    if track_dir:
        os.makedirs(track_dir, exist_ok=True)
        fn = os.path.join(track_dir, f"{map_type}_{map_seed}.json")
        with open(fn, "w") as fh:
            json.dump({
                "map_type": map_type, "seed": map_seed,
                "completed": coord.mission_complete, "steps": steps,
                "columns": ["step", "tx", "ty", "bx", "by", "err", "search"],
                "tracks": tracks,
            }, fh)

    # map warp: cells an agent has DECIDED (<0.4 free / >0.6 wall) that disagree
    # with ground truth, averaged over agents.
    warp_total = 0
    for a in agents:
        warp = 0
        for y in range(map_obj.height):
            for x in range(map_obj.width):
                v = a.internal_map[y][x]
                truth_wall = map_obj.grid[y][x] == 1
                if v > 0.6 and not truth_wall:
                    warp += 1
                elif v < 0.4 and truth_wall:
                    warp += 1
        warp_total += warp

    return {
        "map_type": map_type,
        "seed": map_seed,
        "completed": coord.mission_complete,
        "steps": steps,
        "err_mean": err_sum / max(1, err_n),
        "err_max": err_max,
        "warp": warp_total / max(1, n_agents),
        "elapsed": elapsed,
        "agent_steps": agent_steps,
        "recoveries": n_recoveries,
        "searches": n_searches,
        "erosions": n_erosions,
        "reopens": n_reopens,
        "gates": n_gates,
    }


# =========================
# Seed parsing
# =========================

def parse_seeds(spec):
    """'1-20' or '1000-1099' or '1,3,7' -> list of ints."""
    out = []
    for part in spec.split(","):
        part = part.strip()
        if "-" in part:
            a, b = part.split("-")
            out.extend(range(int(a), int(b) + 1))
        elif part:
            out.append(int(part))
    return out


# =========================
# Reporting
# =========================

def difficulty(r):
    """A scalar difficulty score for seed selection. Non-completion dominates;
    among completers, more steps + higher localization error are harder. Tuned to
    favour seeds that STRESS localization/mapping (the thing the reports test),
    not merely long but easy paths."""
    if not r["completed"]:
        return 10_000 + r["err_mean"] * 100 + r["err_max"]
    return r["steps"] + r["err_mean"] * 200 + r["err_max"] * 20


def main():
    ap = argparse.ArgumentParser(description="Parallel empirical sweep.")
    ap.add_argument("--maps", default="room,bsp,cave",
                    help="comma list of map types")
    ap.add_argument("--seeds", default=None,
                    help="seed spec, e.g. '1-20' or '1000-1099'. "
                         "Default: the curated STANDARD_SEEDS per map.")
    ap.add_argument("--overrides", default="{}",
                    help="JSON config overrides")
    ap.add_argument("--agents", type=int, default=None,
                    help="override num_agents")
    ap.add_argument("--map-size", type=int, default=None,
                    help="override map_width/height (square)")
    ap.add_argument("--max-steps", type=int, default=None,
                    help="override max_steps")
    ap.add_argument("-j", "--jobs", type=int, default=os.cpu_count(),
                    help="parallel worker processes (default: all cores)")
    ap.add_argument("--per-run", action="store_true",
                    help="print one line per run (with recovery diagnostics)")
    ap.add_argument("--track-paths", default=None, metavar="DIR",
                    help="record every agent's per-step trajectory + error +"
                         " search flag to DIR/<map>_<seed>.json (one file per run)")
    ap.add_argument("--select", type=int, default=0, metavar="N",
                    help="seed-selection mode: print the N hardest seeds per map "
                         "(by difficulty score) instead of an aggregate")
    args = ap.parse_args()

    overrides = json.loads(args.overrides)
    if args.agents is not None:
        overrides["num_agents"] = args.agents

    maps = [m.strip() for m in args.maps.split(",") if m.strip()]

    tasks = []
    for m in maps:
        if args.seeds is not None:
            seeds = parse_seeds(args.seeds)
        else:
            seeds = STANDARD_SEEDS.get(m, list(range(1, 21)))
        for s in seeds:
            tasks.append({
                "map_type": m,
                "map_seed": s,
                "behaviour_seed": s,
                "overrides": overrides,
                "map_size": args.map_size,
                "max_steps": args.max_steps,
                "track_paths": args.track_paths,
            })

    t0 = time.perf_counter()
    if args.jobs and args.jobs > 1:
        with Pool(args.jobs) as pool:
            results = pool.map(run_one, tasks)
    else:
        results = [run_one(t) for t in tasks]
    wall = time.perf_counter() - t0

    # ---- selection mode ----
    if args.select:
        by_map = {}
        for r in results:
            by_map.setdefault(r["map_type"], []).append(r)
        print(f"# seed selection (top {args.select} hardest per map)")
        for m in maps:
            rs = sorted(by_map.get(m, []), key=difficulty, reverse=True)
            chosen = [r["seed"] for r in rs[:args.select]]
            print(f'    "{m}": {chosen},')
            for r in rs[:args.select]:
                print(f"      # seed {r['seed']:5d}  "
                      f"{'DONE' if r['completed'] else 'FAIL'}  "
                      f"steps={r['steps']:4d}  err={r['err_mean']:.2f}/"
                      f"{r['err_max']:.2f}  warp={r['warp']:.1f}  "
                      f"diff={difficulty(r):.0f}")
        return

    # ---- aggregate report ----
    n = len(results)
    completed = sum(1 for r in results if r["completed"])
    err_mean = sum(r["err_mean"] for r in results) / n
    err_max = max(r["err_max"] for r in results)
    warp = sum(r["warp"] for r in results) / n
    steps = sum(r["steps"] for r in results) / n
    cpu = sum(r["elapsed"] for r in results)
    asteps = sum(r["agent_steps"] for r in results)
    ms_step = cpu / max(1, asteps) * 1000.0

    per_map = {m: [0, 0] for m in maps}
    fails = []
    for r in results:
        per_map[r["map_type"]][1] += 1
        if r["completed"]:
            per_map[r["map_type"]][0] += 1
        else:
            fails.append(f"{r['map_type']}{r['seed']}")

    if args.per_run:
        for r in sorted(results, key=lambda r: (r["map_type"], r["seed"])):
            print(f"{r['map_type']}{r['seed']:<6} "
                  f"{'DONE' if r['completed'] else 'FAIL'} "
                  f"steps={r['steps']:4d} err={r['err_mean']:.2f}/{r['err_max']:.2f} "
                  f"warp={r['warp']:4.1f} "
                  f"rec={r['recoveries']} srch={r['searches']} "
                  f"ero={r['erosions']} reop={r['reopens']} gate={r['gates']} "
                  f"{r['elapsed']*1000:.0f}ms")

    print(f"overrides: {overrides}")
    print(f"runs: {n}  (maps={maps}, "
          f"{'STANDARD' if args.seeds is None else args.seeds} seeds)")
    print(f"completion: {completed}/{n}")
    for m in maps:
        print(f"  {m}: {per_map[m][0]}/{per_map[m][1]}")
    print(f"loc err mean: {err_mean:.3f}")
    print(f"loc err max:  {err_max:.3f}")
    print(f"map warp mean: {warp:.2f}")
    print(f"avg steps: {steps:.1f}")
    print(f"compute: {ms_step:.3f} ms/agent-step  "
          f"({cpu:.1f}s CPU over {asteps} agent-steps)")
    print(f"wall time: {wall:.1f}s  (-j {args.jobs})")
    # Recovery/"irrational map" diagnostics (totals across all runs).
    print(f"events: recoveries={sum(r['recoveries'] for r in results)} "
          f"searches={sum(r['searches'] for r in results)} "
          f"erosions={sum(r['erosions'] for r in results)} "
          f"reopens={sum(r['reopens'] for r in results)} "
          f"jump-gates={sum(r['gates'] for r in results)}")
    if args.track_paths:
        print(f"paths written to: {args.track_paths}/")
    if fails:
        print(f"failed: {fails}")


if __name__ == "__main__":
    main()
