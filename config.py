"""
config.py

Central configuration for simulation.
"""

CONFIG = {
    # =========================
    # Map
    # =========================
    "map_width": 51,
    "map_height": 51,
    "map_type": "maze",  # "maze", "room", "mixed"

    # =========================
    # Simulation
    # =========================
    "max_steps": 20000,
    "dt": 0.1,

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