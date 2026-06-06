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
    "max_steps": 2000,
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
    },

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