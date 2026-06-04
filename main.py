"""
main.py

Entry point for running the simulation.
"""

from config import CONFIG

from utils.random_manager import RandomManager

from core.environment import Environment
from core.simulation import Simulation

from core.agent import create_agent

# TEMP: using existing generator classes
from maps.maze_generator import MazeGenerator
from maps.room_generator import RoomGenerator
from maps.mixed_generator import MixedGenerator


def create_map(config, rng_manager):
    """
    Create map based on config.
    """

    width = config["map_width"]
    height = config["map_height"]

    map_type = config["map_type"]
    print(f"   Generating {map_type} map ({width}x{height})...")

    if map_type == "maze":
        generator = MazeGenerator(width, height, rng_manager)

    elif map_type == "room":
        generator = RoomGenerator(width, height, rng_manager)

    elif map_type == "mixed":
        generator = MixedGenerator(width, height, rng_manager)

    else:
        raise ValueError(f"Unknown map type: {map_type}")

    return generator.generate()


def main():
    # =========================
    # RNG Setup
    # =========================
    print("1. Initializing RNG Manager...")
    rng_manager = RandomManager(
        map_seed=CONFIG["map_seed"],
        behaviour_seed=CONFIG["behaviour_seed"],
    )

    print("SEEDS:", rng_manager.get_seeds())

    # =========================
    # Map
    # =========================
    print("2. Creating map...")
    map_obj = create_map(CONFIG, rng_manager)
    print("Map created successfully")

    print("\nGenerated Map:\n")
    print(map_obj)

    # =========================
    # Environment
    # =========================
    print("3. Creating environment...")
    environment = Environment(map_obj)
    print("Environment created")

    # =========================
    # Agent
    # =========================
    print("4. Creating agent...")
    start_pos = (map_obj.start[0], map_obj.start[1])
    print(f"   Map start (grid): {map_obj.start}")
    print(f"   Start position (centered): {start_pos}")
    print(f"   Map goal (grid): {map_obj.goal}")

    agent = create_agent(
        agent_id=0,
        start_pos=start_pos,
        map_width=map_obj.width,
        map_height=map_obj.height,
        rng_manager=rng_manager,
        config=CONFIG
    )
    print("Agent created")

    agents = [agent]

    # =========================
    # Simulation
    # =========================
    print("5. Starting simulation...")
    sim = Simulation(environment, agents, CONFIG)
    sim.run()
    print("Simulation completed")


if __name__ == "__main__":
    main()