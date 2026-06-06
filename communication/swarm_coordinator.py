"""
swarm_coordinator.py

A lightweight shared "blackboard" for the cooperative swarm. It holds the
cross-agent state that no single agent owns:

- the single shared occupancy grid (only in map_sharing == "shared"),
- which agent has committed to reaching the goal (sticky once set),
- whether the mission is complete (the goal-reacher is home).

(Frontier allocation is handled directly in exploration/frontier.py via a
proximity partition over the agents, so it needs no shared reservation state.)

One instance is created in main.py, passed to every agent (stored as
``agent.coordinator``) and to the Simulation (for termination).
"""

from typing import Optional


class SwarmCoordinator:
    def __init__(self, shared_map=None, shared_locked=None):
        # Present only in "shared" map mode; all agents reference these grids.
        self.shared_map = shared_map
        self.shared_locked = shared_locked
        # Shared wall-mask version (1-element list, shared by reference) so that
        # in shared-map mode any agent's wall change invalidates every agent's
        # cached distance field. See agents/base_agent.py and core/agent.py.
        self.wallver = [0]

        # Sticky: the id of the agent assigned to reach the goal (None until the
        # goal is discovered and the nearest agent claims it).
        self.goal_claimer: Optional[int] = None

        # Set True when the goal-reacher returns to the start: ends the sim.
        self.mission_complete = False

    def try_claim_goal(self, agent_id: int) -> int:
        """
        Claim the goal for ``agent_id`` if unclaimed. The claim is sticky: once
        an agent owns the goal it keeps it. Returns the current owner.
        """
        if self.goal_claimer is None:
            self.goal_claimer = agent_id
        return self.goal_claimer
