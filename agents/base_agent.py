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
from collections import deque
import math

from utils.debug import dprint


Position = Tuple[float, float]
GridPosition = Tuple[int, int]

# Occupancy above which a cell counts as a wall for localization. Must match
# localization/slam.py's _WALL_THRESHOLD: the SLAM update depends on the map only
# through (cell > WALL_THR), so wall_version need only track crossings of this.
WALL_THR = 0.6


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
        map_update_step: float = 0.1,
        map_lock_high: float = 0.9,
        map_lock_low: float = 0.1,
        coordinator=None,
        communication_range: float = 10.0,
        swarm_slam: bool = False,
        sensor_range_sigma: float = 0.0,
    ):
        self.id = agent_id

        self.radius = radius
        self.speed = speed

        # =========================
        # Multi-agent coordination
        # =========================
        # Shared blackboard (goal claim, frontier reservations, mission state).
        if coordinator is None:
            from communication.swarm_coordinator import SwarmCoordinator
            coordinator = SwarmCoordinator()
        self.coordinator = coordinator
        self.communication_range = communication_range
        self.swarm_slam = swarm_slam
        # Noise std-dev applied to a synthesised inter-agent relative measurement.
        self.sensor_range_sigma = sensor_range_sigma

        # Occupancy-grid update: small steps so a single noisy reading barely
        # moves a cell; a cell locks (stops updating) once it saturates past the
        # high/low thresholds, having seen enough consistent evidence.
        self.map_update_step = map_update_step
        self.map_lock_high = map_lock_high
        self.map_lock_low = map_lock_low

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

        # Cells that have accumulated enough consistent evidence are "locked"
        # and no longer updated, so noise can't flip a confidently-known cell.
        self.locked = [
            [False for _ in range(map_width)]
            for _ in range(map_height)
        ]

        self.map_width = map_width
        self.map_height = map_height

        # Bumped only when the WALL MASK changes - i.e. a cell crosses the
        # localizer's wall threshold (WALL_THR). The SLAM measurement update
        # depends on the map solely through that binary mask, so this lets the
        # distance field be cached across the many steps where cell values drift
        # without any cell flipping wall<->not-wall.
        # Held in a 1-element list so it can be SHARED by reference: in "shared"
        # map mode every agent writes the one grid, so they must share one version
        # (any agent's wall change must invalidate everyone's cached field). In
        # "individual" mode each agent keeps its own. See core/agent.py.
        self._wallver = [0]

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

        # Stuck detection / recovery
        self.stuck_steps = 0
        self.stuck_limit = 25  # consecutive no-progress steps before recovering

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

        # 1. Sense environment (observations are absolute world cells)
        observations = self.sensor_model.sense(
            environment,
            self.true_position
        )

        # 2. Update internal map (uses absolute cells -> map stays world-aligned)
        self._update_internal_map(observations)

        # 3. Update localization.
        # A real range sensor reports hits relative to itself as continuous
        # ranges. Casting from the true *floating-point* position means the scan
        # depends on the sub-tile position, so the filter can localize WITHIN a
        # tile rather than only to tile resolution.
        scan = self.sensor_model.range_scan(environment, self.true_position)

        # Swarm SLAM: a confident in-range neighbour anchors our pose estimate.
        anchors = self._gather_anchors(agents)

        self.believed_position = self.localization_model.update(
            self.believed_position,
            self.actual_motion,
            scan,
            self.internal_map,
            anchors=anchors,
            map_version=self._wallver[0],
        )

        # 4. Planning
        self._plan(environment, agents)

        # 5. Movement
        self._move(dt, environment)

        # Track lack of progress so _plan can trigger recovery if we get stuck.
        if math.hypot(self.actual_motion[0], self.actual_motion[1]) < 1e-6:
            self.stuck_steps += 1
        else:
            self.stuck_steps = 0

        # 6. Communication
        self.communication_model.communicate(self, agents)

        # 7. Check completion
        self._check_goal(environment)

    # =========================
    # Swarm SLAM: inter-agent pose anchoring
    # =========================

    def _gather_anchors(self, agents):
        """
        Build inter-agent pose anchors for collaborative localization.

        For each peer within communication range that is meaningfully more
        confident in its own pose than we are, synthesise a noisy relative
        measurement of that peer (range/bearing style, using true positions as a
        real sensor would) and turn it into an implied estimate of OUR pose:
        ``implied = peer.believed_position - measured_offset``, weighted by the
        peer's confidence. The particle filter folds these in as extra evidence.

        Returns a list of ``(implied_position, weight)`` or None if swarm SLAM is
        disabled, the localizer has no confidence notion, or no peer qualifies.
        """
        if not self.swarm_slam:
            return None
        my_conf_fn = getattr(self.localization_model, "confidence", None)
        if my_conf_fn is None:
            return None  # odometry / exact have no pose confidence to compare
        my_conf = my_conf_fn()

        margin = 0.1  # require a peer to be clearly more confident before trusting it
        ax, ay = self.true_position
        sigma = self.sensor_range_sigma

        anchors = []
        for other in agents:
            if other.id == self.id:
                continue
            other_conf_fn = getattr(other.localization_model, "confidence", None)
            if other_conf_fn is None:
                continue
            bx, by = other.true_position
            if math.hypot(ax - bx, ay - by) > self.communication_range:
                continue
            other_conf = other_conf_fn()
            if other_conf <= my_conf + margin:
                continue
            meas_dx = (bx - ax) + (self.rng.gauss(0, sigma) if sigma > 0 else 0.0)
            meas_dy = (by - ay) + (self.rng.gauss(0, sigma) if sigma > 0 else 0.0)
            implied = (
                other.believed_position[0] - meas_dx,
                other.believed_position[1] - meas_dy,
            )
            anchors.append((implied, other_conf))

        return anchors if anchors else None

    # =========================
    # Internal Map Update
    # =========================

    def _update_internal_map(self, observations):
        """
        Update occupancy grid using sensor observations.

        Each observation nudges a cell by a small step, so a single false
        reading barely moves it - several consistent readings are needed to
        cross the free/wall thresholds. Once a cell saturates past the lock
        thresholds it is locked and no longer updated, immune to later noise.

        observations expected format:
        List of (x, y, occupied: bool)
        """

        step = self.map_update_step
        mask_changed = False

        for x, y, occupied in observations:
            if not (0 <= x < self.map_width and 0 <= y < self.map_height):
                continue
            if self.locked[y][x]:
                continue

            before = self.internal_map[y][x]
            if occupied:
                after = min(1.0, before + step)
            else:
                after = max(0.0, before - step)
            self.internal_map[y][x] = after
            # Only a crossing of the wall threshold affects the SLAM update.
            if (before > WALL_THR) != (after > WALL_THR):
                mask_changed = True

            if after >= self.map_lock_high or after <= self.map_lock_low:
                self.locked[y][x] = True

        if mask_changed:
            self._wallver[0] += 1

    # =========================
    # Planning Logic
    # =========================

    def _plan(self, environment, agents=None):
        """
        Decide where to go next.
        """
        if agents is None:
            agents = [self]
        
        self.steps_since_replan += 1

        agent_cell = self._to_grid(self.believed_position)
        recovery = False  # set when pushing through believed-walls to escape a seal

        # =========================
        # Recovery if frozen too long (any mission phase)
        # =========================
        if self.stuck_steps >= self.stuck_limit:
            self.stuck_steps = 0
            # 0. Unlock every cell so a wrongly-locked value can't keep us stuck;
            #    sensing will re-establish locks as evidence re-accumulates.
            self._unlock_all()
            # 1. Re-open the believed-walls bounding our region. Clears a phantom
            #    (noisy false-positive) wall that sealed us in; real walls get
            #    re-confirmed by sensing.
            self._reopen_boundary(agent_cell)
            # 2. Take one real step into a sensed-free neighbour. This breaks a
            #    physical deadlock (e.g. a wrong pose estimate left us following an
            #    un-followable path, including while heading to the goal/start) and
            #    yields fresh observations to relocalize from.
            step_cell = self._recovery_step(environment)
            if step_cell is not None:
                self.current_target = step_cell
                self.current_path = [step_cell]
                self.steps_since_replan = 0
                return

        # =========================
        # Mission phase target selection
        # =========================
        # 1. After reaching the goal: head back to the start.
        # 2. Once the goal has been discovered (sensed): head straight for it
        #    instead of continuing to explore.
        # 3. Otherwise: keep exploring frontiers.
        if self.returning_to_start:
            self._set_fixed_target(environment.map.start)

        elif self._goal_discovered(environment) and self._claims_goal(agents, environment):
            # We are the swarm's assigned goal-reacher: head straight for it.
            # Every other agent keeps exploring/helping and never targets the goal.
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
                    self.internal_map,
                    agents,
                )
                self.current_path = []
                self.steps_since_replan = 0

                if self.current_target is None:
                    # Still no reachable frontier: head to the nearest unexplored
                    # cell, allowing the path to cross believed-walls.
                    self.current_target = self.exploration_strategy.nearest_unknown(
                        self.internal_map, agent_cell
                    )
                    recovery = self.current_target is not None
                    if self.current_target is None:
                        dprint(f"[Agent {self.id}] No target (exploration complete?)")

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
                self.internal_map,
                allow_walls=recovery
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

    def _reopen_boundary(self, agent_cell) -> int:
        """
        Recovery for being stuck: flood-fill the reachable known-free region and
        reset the believed-walls on its boundary to 'unknown'. This re-opens any
        phantom wall (a noisy false positive) that sealed us in. Cells the sensor
        can still see snap back to wall almost immediately, so real walls are not
        forgotten; the genuine phantom (out of sight) stays open and lets us out.
        Returns how many cells were reopened.
        """
        w, h = self.map_width, self.map_height
        sx, sy = agent_cell
        if not (0 <= sx < w and 0 <= sy < h):
            return 0

        visited = {agent_cell}
        queue = deque([agent_cell])
        boundary = set()

        while queue:
            cx, cy = queue.popleft()
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nx, ny = cx + dx, cy + dy
                if not (0 <= nx < w and 0 <= ny < h) or (nx, ny) in visited:
                    continue
                prob = self.internal_map[ny][nx]
                if prob < 0.4:          # known free -> keep expanding the region
                    visited.add((nx, ny))
                    queue.append((nx, ny))
                elif prob > 0.6:        # believed wall on the boundary
                    boundary.add((nx, ny))

        for bx, by in boundary:
            self.internal_map[by][bx] = 0.5  # reset to unknown

        if boundary:
            # Reopened cells were believed-walls (>0.6) reset to 0.5: mask changed.
            self._wallver[0] += 1

        return len(boundary)

    def _unlock_all(self):
        """Clear all cell locks (recovery safety net)."""
        for row in self.locked:
            for x in range(len(row)):
                row[x] = False

    def _recovery_step(self, environment):
        """
        Pick a free neighbouring cell to physically step into when stuck, to
        break a deadlock and generate fresh observations. Prefers still-unknown
        neighbours so the twitch also makes exploration progress.
        """
        tcx = int(self.true_position[0])
        tcy = int(self.true_position[1])

        free = []
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nx, ny = tcx + dx, tcy + dy
            if environment.is_free(nx, ny):
                free.append((nx, ny))

        if not free:
            return None

        unknown = [
            c for c in free
            if 0.4 <= self.internal_map[c[1]][c[0]] <= 0.6
        ]
        pool = unknown if unknown else free
        return self.rng.choice(pool)

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

    def _claims_goal(self, agents, environment) -> bool:
        """
        Assign the goal to the nearest (by path) still-active agent if it is not
        yet claimed, then report whether THIS agent is the claimer. The claim is
        sticky (held in the coordinator), so once made it does not flip as agents
        move. Only the claimer pursues the goal; everyone else keeps exploring.
        """
        coord = self.coordinator
        if coord.goal_claimer is None:
            goal_cell = environment.map.goal
            best_id, best_d = None, float("inf")
            for a in agents:
                if getattr(a, "finished", False):
                    continue
                d = self._path_distance(
                    a._to_grid(a.believed_position), goal_cell, a.internal_map
                )
                if d < best_d or (d == best_d and (best_id is None or a.id < best_id)):
                    best_d, best_id = d, a.id
            if best_id is not None:
                coord.try_claim_goal(best_id)
        return coord.goal_claimer == self.id

    def _path_distance(self, start_cell, goal_cell, internal_map) -> float:
        """
        BFS step-distance from start_cell to goal_cell over non-wall cells
        (prob < 0.6) on the given map. Returns inf if unreachable.
        """
        if start_cell == goal_cell:
            return 0
        w, h = self.map_width, self.map_height
        sx, sy = start_cell
        if not (0 <= sx < w and 0 <= sy < h):
            return float("inf")

        visited = {start_cell}
        queue = deque([(sx, sy, 0)])
        while queue:
            cx, cy, d = queue.popleft()
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nx, ny = cx + dx, cy + dy
                if not (0 <= nx < w and 0 <= ny < h) or (nx, ny) in visited:
                    continue
                if (nx, ny) == goal_cell:
                    return d + 1
                if internal_map[ny][nx] >= 0.6:  # wall
                    continue
                visited.add((nx, ny))
                queue.append((nx, ny, d + 1))
        return float("inf")

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

        # Only the assigned claimer may "reach" the goal - this is what enforces
        # that a single agent handles the goal and the return trip. Non-claimers
        # may pass over the cell while helping, but it never counts.
        is_claimer = (self.coordinator.goal_claimer == self.id)

        if is_claimer and not self.reached_goal and current_cell == environment.map.goal:
            self.reached_goal = True
            self.returning_to_start = True
            self.current_path = []

        elif self.returning_to_start and current_cell == environment.map.start:
            self.finished = True
            self.coordinator.mission_complete = True

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