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
import random
import sys
import time
from multiprocessing import Pool

from config import CONFIG
from utils.random_manager import RandomManager
from core.environment import Environment
from core.agent import create_agent
from planning.astar import true_path_distance
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

# Curated sets, cut from searches over seeds 2000-23xx with behaviour_seed =
# map_seed and all recovery features ON (room/bsp/cave: 999-run search; maze:
# separate 400-run search). Replay any seed by setting BOTH map_seed and
# behaviour_seed to it. Select with `--set {easy,hard,impossible}` (default hard).
# See docs/TESTING.md. maze is by far the hardest (160/400 complete) - all-
# corridor geometry makes the along-corridor belief offset unobservable.
#
# EASY = fastest completers per map (all < 400 steps; slowest member 66 steps).
# A quick ~100%-completion regression check to run BEFORE the harder sets.
EASY_SEEDS = {
    "room": [2123, 2127, 2147, 2204, 2225, 2321],
    "bsp":  [2013, 2084, 2155, 2168, 2264, 2296],
    "cave": [2035, 2052, 2164, 2166, 2207, 2219],
    "maze": [2056, 2169, 2272, 2275, 2328, 2346],
}

# HARD = slowest completers (balanced per map) + failures spread across "closeness
# to finishing" (75% returning ... 0% never-reached-goal), ~half/half. A
# discriminating regression set: failures can be rescued, slow completers can
# regress into failures. maze adds 5 slow completers + 5 spread failures.
HARD_SEEDS = {
    "room": [2149, 2167, 2219, 2223, 2247, 2260, 2262, 2274, 2297, 2299, 2302, 2315],
    "bsp":  [2065, 2080, 2289, 2327, 2332],
    "cave": [2001, 2008, 2019, 2031, 2038, 2062, 2081, 2155, 2158, 2217, 2251, 2285, 2304],
    "maze": [2022, 2069, 2090, 2138, 2235, 2252, 2266, 2352, 2387, 2391],
}

# IMPOSSIBLE = seeds that FAILED at behaviour=map. room 12, bsp 1, cave 17 (3% of
# the 999-run search) + a representative closeness-spread of 30 maze failures (maze
# failed 240/400 - too many to list all; the persisted slice is sampled across
# closeness, full list reproducible via /tmp searches). Not literally unsolvable
# (most are behaviour-specific), but the genuinely hard cases - use it to hunt
# remaining failure modes and confirm a fix rescues them.
IMPOSSIBLE_SEEDS = {
    "room": [2018, 2069, 2149, 2163, 2167, 2223, 2230, 2262, 2274, 2299, 2302, 2304],
    "bsp":  [2205],
    "cave": [2001, 2010, 2019, 2038, 2062, 2081, 2119, 2134, 2154, 2155,
             2175, 2183, 2190, 2251, 2263, 2285, 2328],
    "maze": [2005, 2009, 2013, 2021, 2042, 2060, 2064, 2075, 2095, 2122,
             2128, 2142, 2159, 2168, 2178, 2197, 2219, 2237, 2261, 2287,
             2294, 2307, 2318, 2326, 2339, 2361, 2371, 2387, 2391, 2396],
}

SEED_SETS = {"easy": EASY_SEEDS, "hard": HARD_SEEDS, "impossible": IMPOSSIBLE_SEEDS}
# Back-compat alias: the default set is the hard set.
STANDARD_SEEDS = HARD_SEEDS


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

    # Mission progress in [0,1] = "how close to finishing" (the mission is
    # start -> goal -> start). 1.0 = completed. For a failure: the first half is
    # reaching the goal, the second half is returning to start, graded by the
    # claimer's remaining distance. Distances are TRUE shortest-path lengths over
    # the known map (an A* oracle), not straight-line: in a maze the goal can be a
    # few cells away straight-line yet hundreds of cells along the only free path,
    # so straight-line progress was badly misleading. `opt_dist` is the one-way
    # oracle distance start->goal (how far the goal actually is).
    sgx, sgy = map_obj.start
    ggx, ggy = map_obj.goal
    opt_dist = true_path_distance(map_obj, (sgx, sgy), (ggx, ggy))
    D = opt_dist if opt_dist not in (0.0, float("inf")) else \
        (math.hypot(sgx - ggx, sgy - ggy) or 1.0)
    claimer = next((a for a in agents if a.id == coord.goal_claimer), None)

    def _remaining(target_cell):
        """Oracle path distance from the claimer's true cell to a target cell,
        falling back to straight-line if it is momentarily unreachable."""
        cell = (int(claimer.true_position[0]), int(claimer.true_position[1]))
        d = true_path_distance(map_obj, cell, target_cell)
        if d == float("inf"):
            d = math.hypot(claimer.true_position[0] - (target_cell[0] + 0.5),
                           claimer.true_position[1] - (target_cell[1] + 0.5))
        return d

    if coord.mission_complete:
        progress = 1.0
        reached_goal = True
    elif claimer is None:
        progress = 0.0          # goal never even discovered/claimed
        reached_goal = False
    elif getattr(claimer, "reached_goal", False):
        rem = _remaining((sgx, sgy))                            # returning leg
        progress = max(0.0, min(1.0, (2 * D - rem) / (2 * D)))
        reached_goal = True
    else:
        rem = _remaining((ggx, ggy))                            # outbound leg
        progress = max(0.0, min(0.5, (D - rem) / (2 * D)))
        reached_goal = False

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
        "progress": progress,
        "reached_goal": reached_goal,
        "opt_dist": opt_dist,
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


def run_batch(tasks, jobs):
    """Run a list of task dicts, in parallel if jobs > 1. Returns (results, wall)."""
    t0 = time.perf_counter()
    if jobs and jobs > 1:
        with Pool(jobs) as pool:
            results = pool.map(run_one, tasks)
    else:
        results = [run_one(t) for t in tasks]
    return results, time.perf_counter() - t0


def summarize(results, maps, label, wall, jobs, per_run=False, track_dir=None):
    """Print the aggregate report block for a batch of results."""
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
        per_map.setdefault(r["map_type"], [0, 0])[1] += 1
        if r["completed"]:
            per_map[r["map_type"]][0] += 1
        else:
            fails.append(f"{r['map_type']}{r['seed']}")

    if per_run:
        for r in sorted(results, key=lambda r: (r["map_type"], r["seed"])):
            od = r.get("opt_dist", float("inf"))
            od_s = f"{od:.0f}" if od != float("inf") else "inf"
            print(f"{r['map_type']}{r['seed']:<6} "
                  f"{'DONE' if r['completed'] else 'FAIL'} "
                  f"steps={r['steps']:4d} opt={od_s:>4} prog={r['progress']:.2f} "
                  f"err={r['err_mean']:.2f}/{r['err_max']:.2f} "
                  f"warp={r['warp']:4.1f} "
                  f"rec={r['recoveries']} srch={r['searches']} "
                  f"ero={r['erosions']} reop={r['reopens']} gate={r['gates']} "
                  f"{r['elapsed']*1000:.0f}ms")

    print(f"--- {label} ---")
    print(f"completion: {completed}/{n}")
    for m in maps:
        print(f"  {m}: {per_map[m][0]}/{per_map[m][1]}")
    print(f"loc err mean: {err_mean:.3f}")
    print(f"loc err max:  {err_max:.3f}")
    print(f"map warp mean: {warp:.2f}")
    print(f"avg steps: {steps:.1f}")
    print(f"compute: {ms_step:.3f} ms/agent-step  "
          f"({cpu:.1f}s CPU over {asteps} agent-steps)")
    print(f"wall time: {wall:.1f}s  (-j {jobs})")
    print(f"events: recoveries={sum(r['recoveries'] for r in results)} "
          f"searches={sum(r['searches'] for r in results)} "
          f"erosions={sum(r['erosions'] for r in results)} "
          f"reopens={sum(r['reopens'] for r in results)} "
          f"jump-gates={sum(r['gates'] for r in results)}")
    if track_dir:
        print(f"paths written to: {track_dir}/")
    if fails:
        print(f"failed: {fails}")


def main():
    ap = argparse.ArgumentParser(description="Parallel empirical sweep.")
    ap.add_argument("--maps", default=None,
                    help="comma list of map types (default: the map keys of the "
                         "selected --set, or room,bsp,cave with explicit --seeds)")
    ap.add_argument("--seeds", default=None,
                    help="seed spec, e.g. '1-20' or '1000-1099'. "
                         "Default: the curated seeds for --set.")
    ap.add_argument("--set", dest="seed_set", default="hard",
                    choices=list(SEED_SETS),
                    help="which curated set to run when --seeds is not given "
                         "(hard | impossible; default hard)")
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
    ap.add_argument("--random", type=int, default=10, metavar="N",
                    help="also run N RANDOM (fresh, never-curated) seeds per map as "
                         "a separate generalization batch - an overfit check, not a "
                         "precise benchmark (seeds differ each run). 0 disables.")
    ap.add_argument("--random-seed", type=int, default=None,
                    help="seed the random-batch seed picker for a reproducible "
                         "generalization batch (default: fresh entropy each run)")
    args = ap.parse_args()

    overrides = json.loads(args.overrides)
    if args.agents is not None:
        overrides["num_agents"] = args.agents

    curated = SEED_SETS[args.seed_set]
    if args.maps:
        maps = [m.strip() for m in args.maps.split(",") if m.strip()]
    elif args.seeds is None:
        maps = list(curated.keys())          # run every map the set covers
    else:
        maps = ["room", "bsp", "cave"]
    tasks = []
    for m in maps:
        if args.seeds is not None:
            seeds = parse_seeds(args.seeds)
        else:
            seeds = curated.get(m, [])
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

    results, wall = run_batch(tasks, args.jobs)

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

    # ---- standard / curated report ----
    print(f"overrides: {overrides}")
    seeds_label = args.seed_set.upper() if args.seeds is None else args.seeds
    summarize(results, maps, f"{seeds_label} seeds ({len(results)} runs)",
              wall, args.jobs, args.per_run, args.track_paths)

    # ---- generalization batch (random, never-curated seeds) ----
    # An overfit sanity check: fresh seeds each run, so it sees maps the solution
    # was never tuned on. Not a precise benchmark (numbers vary run to run); a low
    # completion here when the curated set looks good would flag overfitting.
    if args.random > 0:
        picker = random.Random(args.random_seed)  # entropy-seeded if None
        rand_seeds = {
            m: sorted(picker.sample(range(100_000, 1_000_000), args.random))
            for m in maps
        }
        rtasks = []
        for m in maps:
            for s in rand_seeds[m]:
                rtasks.append({
                    "map_type": m, "map_seed": s, "behaviour_seed": s,
                    "overrides": overrides, "map_size": args.map_size,
                    "max_steps": args.max_steps, "track_paths": args.track_paths,
                })
        rresults, rwall = run_batch(rtasks, args.jobs)
        print()
        summarize(rresults, maps,
                  f"GENERALIZATION: {args.random} RANDOM seeds/map (overfit check)",
                  rwall, args.jobs)
        print(f"random seeds used: {rand_seeds}")


if __name__ == "__main__":
    main()
