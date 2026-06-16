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
    "sensor_range": 3,
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
}