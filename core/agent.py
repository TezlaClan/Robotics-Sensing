"""
agent.py

Provides helper function to construct agents with all required components.
"""

from agents.base_agent import BaseAgent

from sensing.sensor_model import SensorModel
from localization.odometry import OdometryLocalization
from planning.astar import AStarPlanner
from exploration.frontier import FrontierExploration
from communication.communication_model import CommunicationModel


def create_agent(agent_id, start_pos, map_width, map_height, rng_manager, config):
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
        rng_manager=rng_manager
    )

    # =========================
    # Localization (MVP = odometry only)
    # =========================
    localization = OdometryLocalization(
        noise_sigma=config["odometry_noise"],
        rng_manager=rng_manager
    )

    # =========================
    # Exploration
    # =========================
    exploration = FrontierExploration()

    # =========================
    # Planner
    # =========================
    planner = AStarPlanner()

    # =========================
    # Communication (disabled for MVP behaviour)
    # =========================
    communication = CommunicationModel(
        mode="global",
        packet_loss_rate=0.0,
        corruption_rate=0.0,
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
        speed=config["agent_speed"]
    )

    return agent