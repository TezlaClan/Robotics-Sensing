"""
main.py

Entry point for running the simulation.
"""

from config import CONFIG

from utils.debug import set_debug, dprint
from utils.random_manager import RandomManager

from core.environment import Environment
from core.simulation import Simulation

from core.agent import create_agent

# TEMP: using existing generator classes
from maps.maze_generator import MazeGenerator
from maps.room_generator import RoomGenerator
from maps.mixed_generator import MixedGenerator
from maps.cave_generator import CaveGenerator
from maps.obstacle_generator import ObstacleGenerator
from maps.bsp_generator import BSPGenerator


def create_map(config, rng_manager):
    """
    Create map based on config.
    """

    width = config["map_width"]
    height = config["map_height"]

    map_type = config["map_type"]
    dprint(f"   Generating {map_type} map ({width}x{height})...")

    generators = {
        "maze": MazeGenerator,
        "room": RoomGenerator,
        "mixed": MixedGenerator,
        "cave": CaveGenerator,
        "obstacle": ObstacleGenerator,
        "bsp": BSPGenerator,
    }

    if map_type not in generators:
        raise ValueError(
            f"Unknown map type: {map_type}. Options: {', '.join(generators)}"
        )

    generator = generators[map_type](width, height, rng_manager)
    return generator.generate()


def main():
    # Apply verbosity setting before anything prints.
    set_debug(CONFIG.get("debug", False))

    # =========================
    # RNG Setup
    # =========================
    dprint("1. Initializing RNG Manager...")
    rng_manager = RandomManager(
        map_seed=CONFIG["map_seed"],
        behaviour_seed=CONFIG["behaviour_seed"],
    )

    # Record the resolved seeds back into the config so they appear in logs
    # and the recording's setup file (they may have been None / auto-generated).
    seeds = rng_manager.get_seeds()
    CONFIG["map_seed"] = seeds["map_seed"]
    CONFIG["behaviour_seed"] = seeds["behaviour_seed"]
    dprint("SEEDS:", seeds)

    # =========================
    # Map
    # =========================
    dprint("2. Creating map...")
    map_obj = create_map(CONFIG, rng_manager)
    dprint("Map created successfully")

    dprint("\nGenerated Map:\n")
    dprint(map_obj)

    # =========================
    # Environment
    # =========================
    dprint("3. Creating environment...")
    environment = Environment(map_obj)
    dprint("Environment created")

    # =========================
    # Agent
    # =========================
    dprint("4. Creating agent...")
    start_pos = (map_obj.start[0] + 0.5, map_obj.start[1] + 0.5)
    dprint(f"   Map start (grid): {map_obj.start}")
    dprint(f"   Start position (centered): {start_pos}")
    dprint(f"   Map goal (grid): {map_obj.goal}")

    agent = create_agent(
        agent_id=0,
        start_pos=start_pos,
        map_width=map_obj.width,
        map_height=map_obj.height,
        rng_manager=rng_manager,
        config=CONFIG
    )
    dprint("Agent created")

    agents = [agent]

    # =========================
    # Simulation
    # =========================
    dprint("5. Starting simulation...")
    sim = Simulation(environment, agents, CONFIG)
    sim.run()
    dprint("Simulation completed")


if __name__ == "__main__":
    main()