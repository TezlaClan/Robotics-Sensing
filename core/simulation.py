"""
simulation.py

Core simulation loop.

Handles:
- Timestep updates
- Agent processing
- Termination logic
"""

from rendering.renderer import Renderer
from rendering.video_writer import VideoWriter
from utils.debug import dprint, is_debug

class Simulation:
    def __init__(self, environment, agents, config):
        self.environment = environment
        self.agents = agents
        self.config = config

        # Shared blackboard (mission completion is signalled here once the
        # goal-reacher gets home). Read from the agents so callers needn't pass it.
        self.coordinator = agents[0].coordinator if agents else None

        # Build the enabled output methods (live window and/or mp4 recording).
        self.renderers = []
        if config.get("render_live", True):
            self.renderers.append(Renderer(environment, agents, config))
        if config.get("render_video", False):
            self.renderers.append(VideoWriter(environment, agents, config))

        self.max_steps = config["max_steps"]
        self.dt = config["dt"]
        self.render_every = max(1, int(config.get("render_every", 1)))

        # Metrics
        self.current_step = 0

    # =========================
    # Main Run Loop
    # =========================

    def run(self):
        """
        Run simulation until termination.
        """

        dprint("Starting simulation...")

        while not self._should_terminate():
            self._step()

        # Finalize outputs (e.g. flush/save the mp4).
        for renderer in self.renderers:
            try:
                renderer.close()
            except Exception as e:
                print(f"ERROR closing renderer: {e}")

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

        # Progress reporting:
        #   debug off -> just the step counter every 50 steps
        #   debug on  -> detailed per-agent block (and first 10 steps)
        if is_debug():
            if self.current_step % 50 == 0 or self.current_step <= 10:
                self._log_progress()
        elif self.current_step % 50 == 0:
            print(f"Step {self.current_step}")

        # Render only every Nth step (render_every); always render the first step.
        if self.current_step % self.render_every == 0:
            for renderer in self.renderers:
                try:
                    renderer.render()
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

        # 2. Mission complete: the goal-reacher has returned to start. Non-reacher
        #    agents never set `finished`, so this is the normal cooperative end.
        if self.coordinator is not None and self.coordinator.mission_complete:
            print("Mission complete: goal reached and returned to start.")
            return True

        # 3. All agents finished (fallback, e.g. single-agent runs)
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