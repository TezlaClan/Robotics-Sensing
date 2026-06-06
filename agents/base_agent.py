"""
base_agent.py

Defines the core Agent class.

Responsibilities:
- Maintain true position and belief position
- Maintain internal occupancy grid map
- Interface with:
    - sensor model
    - localization model
    - exploration strategy
    - path planner
    - communication model
"""

from typing import Tuple, List, Optional
import math

from utils.debug import dprint


Position = Tuple[float, float]
GridPosition = Tuple[int, int]


class BaseAgent:
    def __init__(
        self,
        agent_id: int,
        start_pos: Position,
        map_width: int,
        map_height: int,
        sensor_model,
        localization_model,
        exploration_strategy,
        planner,
        communication_model,
        rng_manager,
        radius,
        speed,
    ):
        self.id = agent_id

        self.radius = radius
        self.speed = speed

        # =========================
        # True State
        # =========================
        self.true_position: Position = start_pos
        self.velocity: Position = (0.0, 0.0)
        self.actual_motion = (0.0, 0.0)

        # =========================
        # Belief State
        # =========================
        self.believed_position: Position = start_pos

        # =========================
        # Internal Map (Occupancy Grid)
        # Probabilities:
        #   0.0 = definitely free
        #   1.0 = definitely wall
        #   0.5 = unknown
        # =========================
        self.internal_map = [
            [0.5 for _ in range(map_width)]
            for _ in range(map_height)
        ]

        self.map_width = map_width
        self.map_height = map_height

        # =========================
        # Modules
        # =========================
        self.sensor_model = sensor_model
        self.localization_model = localization_model
        self.exploration_strategy = exploration_strategy
        self.planner = planner
        self.communication_model = communication_model

        self.rng_manager = rng_manager
        self.rng = rng_manager.behaviour_rng()

        # =========================
        # Planning / Navigation
        # =========================
        self.current_path: List[GridPosition] = []
        self.current_target: Optional[GridPosition] = None
        self.steps_since_replan = 0
        self.replan_interval = 5  # Replan every 5 steps

        # =========================
        # Status Flags
        # =========================
        self.reached_goal = False
        self.returning_to_start = False
        self.finished = False

    # =========================
    # Main Update Loop
    # =========================

    def step(self, environment, agents: List["BaseAgent"], dt: float):
        """
        Main per-timestep update.
        """

        # 1. Sense environment
        observations = self.sensor_model.sense(
            environment,
            self.true_position
        )

        # 2. Update internal map
        self._update_internal_map(observations)

        # 3. Update localization
        self.believed_position = self.localization_model.update(
            self.believed_position,
            self.actual_motion,
            observations
        )

        # 4. Planning
        self._plan(environment)

        # 5. Movement
        self._move(dt, environment)

        # 6. Communication
        self.communication_model.communicate(self, agents)

        # 7. Check completion
        self._check_goal(environment)

    # =========================
    # Internal Map Update
    # =========================

    def _update_internal_map(self, observations):
        """
        Update occupancy grid using sensor observations.

        observations expected format:
        List of (x, y, occupied: bool)
        """

        for x, y, occupied in observations:
            if 0 <= x < self.map_width and 0 <= y < self.map_height:
                if occupied:
                    self.internal_map[y][x] = min(1.0, self.internal_map[y][x] + 0.2)
                else:
                    self.internal_map[y][x] = max(0.0, self.internal_map[y][x] - 0.2)

    # =========================
    # Planning Logic
    # =========================

    def _plan(self, environment):
        """
        Decide where to go next.
        """
        
        self.steps_since_replan += 1

        agent_cell = self._to_grid(self.believed_position)

        # =========================
        # Mission phase target selection
        # =========================
        # 1. After reaching the goal: head back to the start.
        # 2. Once the goal has been discovered (sensed): head straight for it
        #    instead of continuing to explore.
        # 3. Otherwise: keep exploring frontiers.
        if self.returning_to_start:
            self._set_fixed_target(environment.map.start)

        elif self._goal_discovered(environment):
            self._set_fixed_target(environment.map.goal)

        else:
            # =========================
            # Frontier exploration (with commitment)
            # =========================
            # Keep heading to the current target until we actually reach it or it
            # stops being a frontier (its unknown neighbours have been sensed).
            # Re-picking a target every replan causes oscillation: the "nearest"
            # frontier flips as the agent moves between two of them.
            target_still_valid = (
                self.current_target is not None and
                agent_cell != self.current_target and
                self._target_is_frontier(self.current_target)
            )

            if not target_still_valid:
                self.current_target = self.exploration_strategy.choose_target(
                    self,
                    self.internal_map
                )
                self.current_path = []
                self.steps_since_replan = 0

                if self.current_target is None:
                    dprint(f"[Agent {self.id}] No target selected by exploration strategy")

        # =========================
        # Path (re)planning to the committed target
        # =========================
        # Recompute the path periodically or when we don't have one, but do NOT
        # change the target here.
        need_path = (
            not self.current_path or
            self.steps_since_replan >= self.replan_interval
        )

        if need_path and self.current_target:
            self.steps_since_replan = 0
            dprint(f"[Agent {self.id}] Planning: start={agent_cell}, target={self.current_target}")

            self.current_path = self.planner.plan(
                agent_cell,
                self.current_target,
                self.internal_map
            )

            if self.current_path:
                dprint(f"[Agent {self.id}] Path found: {len(self.current_path)} cells")
            else:
                dprint(f"[Agent {self.id}] No path found!")

    def _target_is_frontier(self, target) -> bool:
        """
        Ask the exploration strategy whether `target` is still a frontier.
        Falls back to True for strategies that don't expose this check.
        """
        checker = getattr(self.exploration_strategy, "is_frontier", None)
        if checker is None:
            return True
        return checker(self.internal_map, target)

    def _set_fixed_target(self, cell):
        """
        Lock navigation onto a fixed cell (the goal or start). Resets the path
        only when the target actually changes, so direct navigation replans on
        the normal interval rather than every step.
        """
        if self.current_target != cell:
            self.current_target = cell
            self.current_path = []
            self.steps_since_replan = 0

    def _goal_discovered(self, environment) -> bool:
        """
        True once the goal cell has been sensed as free in the internal map,
        i.e. the agent has actually "found" the goal during exploration.
        """
        gx, gy = environment.map.goal
        if not (0 <= gx < self.map_width and 0 <= gy < self.map_height):
            return False
        return self.internal_map[gy][gx] < 0.4

    # =========================
    # Movement Logic
    # =========================

    def _move(self, dt: float, environment):
      """
      Move along planned path with continuous motion.
      """

      # =========================
      # 1. Path validity check
      # =========================
      if self.current_path:
          next_cell = self.current_path[0]
          x, y = next_cell

          if not environment.is_free(x, y):
              dprint(f"[Agent {self.id}] Path blocked at {next_cell}, clearing path")
              self.current_path = []

      # =========================
      # 2. No path → stop
      # =========================
      if not self.current_path:
          self.velocity = (0.0, 0.0)
          self.actual_motion = (0.0, 0.0)
          return

      # =========================
      # 3. Target cell
      # =========================
      target_cell = self.current_path[0]
      target_pos = self._cell_to_world(target_cell)

      old_x, old_y = self.true_position

      dx = target_pos[0] - old_x
      dy = target_pos[1] - old_y

      dist = math.hypot(dx, dy)

      # =========================
      # 4. Close enough → advance path
      # =========================
      if dist < 0.1:
          dprint(f"[Agent {self.id}] Reached waypoint {target_cell}, advancing path ({len(self.current_path)} remaining)")
          self.current_path.pop(0)
          self.velocity = (0.0, 0.0)
          self.actual_motion = (0.0, 0.0)
          return

      # =========================
      # 5. Desired velocity
      # =========================
      vx = (dx / dist) * self.speed
      vy = (dy / dist) * self.speed

      # =========================
      # 6. Proposed movement
      # =========================
      new_x = old_x + vx * dt
      new_y = old_y + vy * dt

      # Default: no movement
      final_x, final_y = old_x, old_y
      moved = False

      # =========================
      # 7. Collision handling
      # =========================

      # Try full movement
      if self._is_valid_position(new_x, new_y, environment):
          final_x, final_y = new_x, new_y
          moved = True

      else:
          moved = False

          # Try X axis only (slide)
          if self._is_valid_position(new_x, old_y, environment):
              final_x = new_x
              moved = True

          # Try Y axis only (slide)
          elif self._is_valid_position(old_x, new_y, environment):
              final_y = new_y
              moved = True
          
          if not moved:
              dprint(f"[Agent {self.id}] BLOCKED: Cannot move from ({old_x:.1f}, {old_y:.1f}) toward waypoint {target_cell}")

      # =========================
      # 8. Apply movement
      # =========================
      self.true_position = (final_x, final_y)

      # =========================
      # 9. ACTUAL motion (CRITICAL FIX)
      # =========================
      actual_dx = final_x - old_x
      actual_dy = final_y - old_y

      self.actual_motion = (actual_dx, actual_dy)

      # =========================
      # 10. Store velocity (commanded)
      # =========================
      self.velocity = (vx, vy)

    # =========================
    # Goal Logic
    # =========================

    def _check_goal(self, environment):
        current_cell = self._to_grid(self.true_position)

        if not self.reached_goal and current_cell == environment.map.goal:
            self.reached_goal = True
            self.returning_to_start = True
            self.current_path = []

        elif self.returning_to_start and current_cell == environment.map.start:
            self.finished = True

    # =========================
    # Coordinate Helpers
    # =========================

    def _to_grid(self, pos: Position) -> GridPosition:
        return (int(pos[0]), int(pos[1]))

    def _cell_to_world(self, cell: GridPosition) -> Position:
        return (cell[0] + 0.5, cell[1] + 0.5)
    
    def _is_valid_position(self, x, y, environment):
      """
      Check if a circular agent can occupy this position.
      """

      r = self.radius

      # Check surrounding cells within radius
      min_x = int(x - r)
      max_x = int(x + r)
      min_y = int(y - r)
      max_y = int(y + r)

      for cy in range(min_y, max_y + 1):
          for cx in range(min_x, max_x + 1):

              if not environment.map.in_bounds(cx, cy):
                  return False

              if not environment.is_free(cx, cy):

                  # Check distance from circle center to cell center
                  cell_center_x = cx + 0.5
                  cell_center_y = cy + 0.5

                  dist = ((x - cell_center_x) ** 2 + (y - cell_center_y) ** 2) ** 0.5

                  if dist < r + 0.5:
                      return False

      return True