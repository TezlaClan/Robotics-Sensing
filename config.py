"""
config.py

Central configuration for simulation.
"""

CONFIG = {
    # =========================
    # Map
    # =========================
    "map_width": 41,
    "map_height": 41,
    # "maze", "room", "mixed", "cave", "obstacle", "bsp"
    "map_type": "maze",

    # =========================
    # Simulation
    # =========================
    "max_steps": 4000,
    "dt": 0.1,

    # Verbose console tracing (planner / exploration / agent / setup).
    # When False, only the periodic step counter and errors are shown.
    "debug": False,

    # =========================
    # Rendering
    # =========================
    # Output methods (enable one or both)
    "render_live": True,         # interactive matplotlib window
    "render_video": False,       # write an mp4 recording of the run

    # Draw/record a frame only every Nth simulation step (1 = every step).
    # Higher values speed up runs by skipping rendering work (mainly the live
    # window); the simulation itself still steps every step.
    "render_every": 1,

    # Video options (used when render_video is True)
    # Recordings are auto-named "<N>_<map>_<localization>_<agents>_<comm>.mp4"
    # inside video_dir, each with a matching .json setup file.
    "video_dir": "videos",
    "video_fps": 30,
    "video_dpi": 100,

    # Which visual layers are drawn (applies to BOTH live and video output)
    "render_layers": {
        "map": True,         # occupancy grid (walls / free space)
        "fog": True,         # fog-of-war from each agent's belief
        "tint": True,        # red/green tint on known wall/free cells
        "start_goal": True,  # start and goal markers
        "path": True,        # planned A* path
        "particles": True,   # SLAM particle cloud (orange), if using slam
        "agents": True,      # true (red) and believed (blue) agent positions
        "links": True,       # faint lines between in-range (communicating) agents
        "drift": True,       # map-drift overlay: magenta=displaced wall, cyan=vacated wall
    },

    # =========================
    # Multi-agent
    # =========================
    "num_agents": 3,             # number of cooperative agents in the swarm
    # How agents share what they map:
    #   "shared"     -> all agents read/write ONE global occupancy grid
    #   "individual" -> each agent keeps its own grid, merged only when in range
    "map_sharing": "individual",

    # How each agent's occupancy grid is anchored:
    #   "world" -> integrate observations at their true world cells; the map is
    #              globally aligned to ground truth (clean benchmark; default).
    #   "local" -> integrate at the robot's BELIEVED pose (real SLAM-style): the
    #              map rides the estimated trajectory and drifts/warps with the
    #              localization error (no loop closure pulls it back). Intended for
    #              "individual" maps - it is not meaningful with a single shared
    #              grid, since agents have different believed frames.
    "map_anchor": "local",

    # Communication (only meaningful when map_sharing == "individual")
    "comm_mode": "local",        # "global" (always) or "local" (within range only)
    "communication_range": 5.0, # cells; range for "local" comms and swarm anchoring
    "comm_packet_loss": 0.1,     # chance a map transmission is dropped
    "comm_corruption": 0.0,      # per-cell chance a transmitted value is corrupted

    # Swarm SLAM: a confident agent's pose anchors a less-confident neighbour
    # in range (collaborative localization).
    "swarm_slam": True,
    "slam_anchor_sigma": 1.0,    # cells; spread of an inter-agent relative measurement

    # =========================
    # Sensor
    # =========================
    "sensor_range": 8,
    "sensor_mode": "radius",  # "radius" or "los"
    "sensor_false_positive": 0.1,   # free cell reported as an obstacle
    "sensor_false_negative": 0.1,   # obstacle reported as free
    "sensor_range_sigma": 0.1,      # Gaussian range-finding error (cells) on hits
    "sensor_range_outlier_rate": 0.1,  # chance of a gross range outlier per hit
    "sensor_num_beams": 72,         # rays cast for the localization range scan

    # =========================
    # Localization
    # =========================
    "localization": "slam",      # "odometry", "slam", "exact"
    "odometry_noise": 0.1,       # per-step drift std-dev for the "odometry" localizer
    # SLAM = Monte Carlo Localization (particle filter)
    "slam_motion_sigma": 0.02,   # particle process noise (keep small: too much makes
                                 #   the estimate wander along corridors)
    "slam_num_particles": 200,   # particle cloud size
    "slam_measurement_sigma": 1.2,  # likelihood-field std-dev in cells (larger = more forgiving)
    "slam_z_rand": 0.1,          # uniform-noise floor for outlier robustness (0..1)
    # Spurious-jump gating: an estimate that leaps more than |motion| + this many
    # cells in one step is treated as a perceptual-aliasing teleport, rejected,
    # and the cloud re-seeded at the motion-predicted pose (the map is left
    # intact). Larger = more tolerant of big single-step corrections.
    "slam_jump_margin": 2.0,
    # Local map mode only (EXPERIMENTAL, default off): extra likelihood weight for
    # scan hits landing on TRUSTED (locked) walls, as an explicit global drift
    # anchor. In practice the filter already localizes against locked walls (they
    # are part of the >wall-threshold set), so at sensor_range 8 this re-weighting
    # does not beat the baseline - small values (~0.2) cut worst-case drift but
    # raise average error; larger values degrade it. Kept as an opt-in lever.
    "slam_trusted_weight": 0.0,
    # Speed/accuracy knob: scan endpoints scored per particle per step. Lower is
    # faster with graceful accuracy loss (see also slam_num_particles above and
    # sensor_num_beams in the Sensor section).
    "slam_max_endpoints": 24,

    # =========================
    # Mapping (occupancy grid)
    # =========================
    # Each observation nudges a cell by map_update_step (smaller = more evidence
    # needed, more robust to false readings). A cell locks (stops updating) once
    # it saturates at/above map_lock_high (wall) or at/below map_lock_low (free).
    "map_update_step": 0.1,
    "map_lock_high": 0.9,
    "map_lock_low": 0.1,
    # Lock erosion: normally a locked cell is frozen forever. With this on, a
    # locked cell observed CONTRADICTING its locked state (a locked wall seen
    # free, or a locked-free cell seen as wall) for `lock_erosion_patience`
    # consecutive steps is unlocked and reset to unknown, so it can re-heal.
    # Mainly for "local" map mode, where localization drift can place and freeze
    # phantom walls; requiring consecutive contradictions keeps stray sensor
    # noise from eroding correct cells.
    "lock_erosion": True,
    "lock_erosion_patience": 5,

    # =========================
    # Seeds
    # =========================
    "map_seed": None,
    "behaviour_seed": None,

    # =========================
    # Agent
    # =========================
    "agent_radius": 0.25,
    "agent_speed": 2,

    # =========================
    # Recovery / search mode
    # =========================
    # When forward progress stalls because a (often phantom, local-mode) wall
    # seals the way, the agent can oscillate in place: it keeps "moving" a little,
    # so the zero-motion stuck timer never fires. Search mode catches this by
    # tracking NET displacement of the believed pose over a window: if the agent
    # has not netted `search_min_progress` cells over the last `search_window`
    # steps, it enters a survey mode and heads to the FURTHEST reachable frontier,
    # pathing around the known region so its boundary walls (incl. the phantom)
    # get re-observed from new angles. Combined with lock erosion, that lets the
    # phantom flip free again, after which it resumes onward. Mainly useful in
    # "local" map mode.
    "search_recovery": True,
    "search_window": 30,          # steps of believed-pose history for the detector
    "search_min_progress": 1.5,   # cells of net displacement below which = stalled
    "search_block_frac": 0.5,     # min fraction of blocked steps in the window to
                                  # count as a genuine seal (vs slow maneuvering)
    "search_linger": 6,           # steps to dwell at a survey target so the sealing
                                  # wall is observed long enough to erode (a single
                                  # drive-by barely glimpses it). A target lingered
                                  # at without opening a wall is not revisited, so a
                                  # real wall is tried once then search moves on.

    # Multi-agent erosion protection: after we erode a (phantom) wall, a peer that
    # still holds it locked would re-impose and re-lock it on the next map merge,
    # undoing the erosion (worse the more agents are nearby). Cells we eroded stay
    # protected from a peer re-locking a wall there for this many steps.
    "erosion_protect_steps": 30,

    # Navigation: when True the A* planner blocks only on LOCKED (confirmed) walls;
    # an unlocked >wall cell (still accumulating evidence, e.g. a transient phantom)
    # is passable but penalised. A wall must be confirmed before it can seal a
    # route, so phantoms that never lock never seal; real walls lock and block.
    "nav_locked_only": True,

    # Map merge: when an agent has a cell locked as a WALL that a peer confidently
    # sees as free, distrust it and reset to unknown so sensing re-decides. EXPERIMENTAL,
    # default OFF: measured net-negative on the hard set (16/30 vs 17/30 with it off,
    # and it drags nav_locked_only down 17->16). Root cause is the local-mode frame
    # mismatch - agents in different believed frames disagree on indices legitimately,
    # so reconsidering removes correct geometry more often than it clears a phantom.
    # Kept as an opt-in lever (it does cut worst-case drift: loc max 21->12).
    "merge_reconsider": False,

    # Occlusion gating: a confirmed (locked) wall blocks sensing past it. The cell
    # FOV is already shadowcast against the true map, but in "local" map mode an
    # observation is re-anchored to the believed pose, so a drift offset can place
    # a genuine reading BEHIND a believed locked wall and corrupt it. When on, any
    # observation whose ray from the believed pose crosses a locked wall is dropped
    # (not allowed to change the map). The wall being looked at is still mapped;
    # only cells beyond it are blocked.
    "occlusion_block": True,
}