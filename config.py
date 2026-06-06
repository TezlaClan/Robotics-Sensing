"""
config.py

Central configuration for simulation.
"""

CONFIG = {
    # =========================
    # Map
    # =========================
    "map_width": 31,
    "map_height": 31,
    "map_type": "maze",  # "maze", "room", "mixed"

    # =========================
    # Simulation
    # =========================
    "max_steps": 20000,
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
        "agents": True,      # true (red) and believed (blue) agent positions
    },

    # =========================
    # Sensor
    # =========================
    "sensor_range": 5,
    "sensor_mode": "radius",  # "radius" or "los"
    "sensor_false_positive": 0.0,
    "sensor_false_negative": 0.0,

    # =========================
    # Localization
    # =========================
    "odometry_noise": 0.0,

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