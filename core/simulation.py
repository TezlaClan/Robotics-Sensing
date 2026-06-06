"""
simulation.py

Core simulation loop.

Handles:
- Timestep updates
- Agent processing
- Termination logic
"""

from rendering.renderer import Renderer

class Simulation:
    def __init__(self, environment, agents, config):
        self.environment = environment
        self.agents = agents
        self.config = config
        self.renderer = Renderer(environment, agents)

        self.max_steps = config["max_steps"]
        self.dt = config["dt"]

        # Metrics
        self.current_step = 0

    # =========================
    # Main Run Loop
    # =========================

    def run(self):
        """
        Run simulation until termination.
        """

        print("Starting simulation...")

        while not self._should_terminate():
            self._step()

        print("\nSimulation finished.")
        print(f"Total steps: {self.current_step}")

    # =========================
    # Single Step
    # =========================

    def _step(self):
        """
        Execute one timestep.
        """

        self.current_step += 1

        for agent in self.agents:
            try:
                if hasattr(agent, 'finished') and agent.finished:
                    continue
                agent.step(
                    self.environment,
                    self.agents,
                    self.dt
                )
            except Exception as e:
                print(f"ERROR in agent.step(): {e}")
                import traceback
                traceback.print_exc()
                raise

        # Print progress periodically
        if self.current_step % 50 == 0 or self.current_step <= 10:
            self._log_progress()

        try:
            self.renderer.render()
        except Exception as e:
            print(f"ERROR in renderer.render(): {e}")
            import traceback
            traceback.print_exc()

    # =========================
    # Termination Conditions
    # =========================

    def _should_terminate(self) -> bool:
        # 1. Max steps
        if self.current_step >= self.max_steps:
            print("Reached max steps.")
            return True

        # 2. All agents finished
        try:
            if all(agent.finished for agent in self.agents):
                print("All agents completed objective.")
                return True
        except AttributeError as e:
            print(f"ERROR: agent.finished not found - {e}")
            return True

        return False

    # =========================
    # Debug Logging
    # =========================

    def _log_progress(self):
        print(f"\nStep {self.current_step}")

        for agent in self.agents:
            pos = agent.true_position
            belief = agent.believed_position

            status = "FINISHED" if agent.finished else "ACTIVE"

            print(
                f"[Agent {agent.id}] "
                f"True={pos} "
                f"Believed={belief} "
                f"State={status}"
            )