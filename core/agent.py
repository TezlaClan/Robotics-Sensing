"""
agent.py

Provides helper function to construct agents with all required components.
"""

from agents.base_agent import BaseAgent

from sensing.sensor_model import SensorModel
from localization.odometry import OdometryLocalization
from localization.slam import SLAMLocalization
from localization.exact import ExactLocalization
from planning.astar import AStarPlanner
from exploration.frontier import FrontierExploration
from communication.communication_model import CommunicationModel


def _create_localization(config, rng_manager):
    """Build the localization model selected by config["localization"]."""
    kind = config.get("localization", "odometry")

    if kind == "odometry":
        return OdometryLocalization(
            noise_sigma=config["odometry_noise"],
            rng_manager=rng_manager,
        )
    elif kind == "slam":
        return SLAMLocalization(
            noise_sigma=config.get("slam_motion_sigma", 0.05),
            sensor_range=config["sensor_range"],
            num_particles=config.get("slam_num_particles", 200),
            measurement_sigma=config.get("slam_measurement_sigma", 1.2),
            z_rand=config.get("slam_z_rand", 0.1),
            anchor_sigma=config.get("slam_anchor_sigma", 1.0),
            max_endpoints=config.get("slam_max_endpoints", 24),
            jump_margin=config.get("slam_jump_margin", 2.0),
            trusted_weight=config.get("slam_trusted_weight", 0.0),
            rng_manager=rng_manager,
        )
    elif kind == "exact":
        return ExactLocalization(rng_manager=rng_manager)
    else:
        raise ValueError(
            f"Unknown localization: {kind}. Options: odometry, slam, exact"
        )


def create_agent(agent_id, start_pos, map_width, map_height, rng_manager, config,
                 coordinator=None):
    """
    Factory method to create a fully configured agent.
    """

    # =========================
    # Sensor
    # =========================
    sensor = SensorModel(
        sensor_range=config["sensor_range"],
        mode=config["sensor_mode"],
        false_positive_rate=config["sensor_false_positive"],
        false_negative_rate=config["sensor_false_negative"],
        range_sigma=config.get("sensor_range_sigma", 0.0),
        range_outlier_rate=config.get("sensor_range_outlier_rate", 0.0),
        num_beams=config.get("sensor_num_beams", 72),
        rng_manager=rng_manager
    )

    # =========================
    # Localization (odometry / slam / exact)
    # =========================
    localization = _create_localization(config, rng_manager)

    # =========================
    # Exploration
    # =========================
    exploration = FrontierExploration(
        gain_weight=config.get("frontier_gain_weight", 0.0)
    )

    # =========================
    # Planner
    # =========================
    planner = AStarPlanner(
        nav_locked_only=config.get("nav_locked_only", False),
        wall_affinity=config.get("nav_wall_affinity", False),
        wall_affinity_weight=config.get("nav_wall_affinity_weight", 1.0),
        wall_affinity_comfort=config.get("nav_wall_affinity_comfort", None),
        sensor_range=config["sensor_range"],
    )

    # =========================
    # Communication
    # =========================
    # In "shared" map mode every agent writes the same grid, so map exchange is
    # redundant -> "off". Otherwise honour the configured mode/range/noise.
    if config.get("map_sharing", "individual") == "shared":
        comm_mode = "off"
    else:
        comm_mode = config.get("comm_mode", "local")

    communication = CommunicationModel(
        mode=comm_mode,
        communication_range=config.get("communication_range", 10.0),
        packet_loss_rate=config.get("comm_packet_loss", 0.0),
        corruption_rate=config.get("comm_corruption", 0.0),
        merge_reconsider=config.get("merge_reconsider", False),
        rng_manager=rng_manager
    )

    # =========================
    # Create agent
    # =========================
    agent = BaseAgent(
        agent_id=agent_id,
        start_pos=start_pos,
        map_width=map_width,
        map_height=map_height,
        sensor_model=sensor,
        localization_model=localization,
        exploration_strategy=exploration,
        planner=planner,
        communication_model=communication,
        rng_manager=rng_manager,
        radius=config["agent_radius"],
        speed=config["agent_speed"],
        map_update_step=config.get("map_update_step", 0.1),
        map_lock_high=config.get("map_lock_high", 0.9),
        map_lock_low=config.get("map_lock_low", 0.1),
        coordinator=coordinator,
        communication_range=config.get("communication_range", 10.0),
        swarm_slam=config.get("swarm_slam", False),
        sensor_range_sigma=config.get("sensor_range_sigma", 0.0),
        map_anchor=config.get("map_anchor", "world"),
        lock_erosion=config.get("lock_erosion", False),
        lock_erosion_patience=config.get("lock_erosion_patience", 5),
        search_recovery=config.get("search_recovery", True),
        search_window=config.get("search_window", 30),
        search_min_progress=config.get("search_min_progress", 1.5),
        search_block_frac=config.get("search_block_frac", 0.5),
        search_linger=config.get("search_linger", 6),
        erosion_protect_steps=config.get("erosion_protect_steps", 30),
        occlusion_block=config.get("occlusion_block", True),
        stuck_progress_window=config.get("stuck_progress_window", 80),
        stuck_progress_min=config.get("stuck_progress_min", 2.0),
    )

    # In "shared" map mode all agents reference the one grid held by the
    # coordinator, so every observation accumulates into a single global map.
    # They must also share one wall-mask version, so any agent's wall change
    # invalidates every agent's cached distance field.
    if (config.get("map_sharing", "individual") == "shared"
            and coordinator is not None and coordinator.shared_map is not None):
        agent.internal_map = coordinator.shared_map
        agent.locked = coordinator.shared_locked
        agent._wallver = coordinator.wallver

    return agent