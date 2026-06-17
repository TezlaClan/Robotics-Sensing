"""
communication_model.py

Handles multi-agent communication.

Supports:
- Global communication
- Proximity-based communication
- Packet loss
- Data corruption
"""

from typing import List
import math


class CommunicationModel:
    def __init__(
        self,
        mode: str = "global",  # "global" or "local"
        communication_range: float = 10.0,
        packet_loss_rate: float = 0.0,
        corruption_rate: float = 0.0,
        merge_reconsider: bool = False,
        rng_manager=None,
    ):
        self.mode = mode
        self.communication_range = communication_range

        self.packet_loss_rate = packet_loss_rate
        self.corruption_rate = corruption_rate
        # When two agents both have a cell LOCKED but disagree (wall vs free),
        # distrust both: unlock and reset it so fresh sensing re-decides (and must
        # re-accumulate before re-locking). Forces reconsideration of a contested
        # wall instead of each agent silently keeping its own.
        self.merge_reconsider = merge_reconsider

        self.rng_manager = rng_manager
        self.rng = rng_manager.behaviour_rng() if rng_manager else None

    # =========================
    # Main API
    # =========================

    def communicate(self, agent, agents: List):
        """
        Share information between agents.
        """

        # "off" -> sharing handled elsewhere (e.g. a single shared map).
        if self.mode == "off":
            return

        for other in agents:
            if other.id == agent.id:
                continue

            if not self._can_communicate(agent, other):
                continue

            # Packet loss check
            if self._packet_lost():
                continue

            # Send map (optionally corrupted in transit)
            received_map = self._transmit_map(other.internal_map)

            # Merge: adopt the peer's CONFIDENT (locked) cells only. Cells we have
            # recently eroded are protected, so a peer that still holds the old
            # (phantom) wall can't re-impose and re-lock it and undo our erosion.
            mask_changed = self._merge_maps(
                agent.internal_map, agent.locked,
                received_map, other.locked,
                protect=getattr(agent, "_eroded_cooldown", None),
            )
            # Wall mask changed -> invalidate the localizer's cached distance field.
            if mask_changed and hasattr(agent, "_wallver"):
                agent._wallver[0] += 1

    # =========================
    # Communication Rules
    # =========================

    def _can_communicate(self, agent_a, agent_b) -> bool:
        """
        Determines whether two agents can communicate.
        """

        if self.mode == "global":
            return True

        elif self.mode == "local":
            return self._within_range(agent_a, agent_b)

        return False

    def _within_range(self, agent_a, agent_b) -> bool:
        ax, ay = agent_a.true_position
        bx, by = agent_b.true_position

        dist = math.hypot(ax - bx, ay - by)
        return dist <= self.communication_range

    # =========================
    # Transmission Effects
    # =========================

    def _packet_lost(self) -> bool:
        if self.rng is None:
            return False
        return self.rng.random() < self.packet_loss_rate

    def _transmit_map(self, internal_map):
        """
        Apply corruption to transmitted data.
        """

        if self.rng is None or self.corruption_rate == 0:
            return internal_map

        corrupted = []

        for row in internal_map:
            new_row = []
            for value in row:

                if self.rng.random() < self.corruption_rate:
                    # Corrupt value randomly
                    value = self.rng.random()

                new_row.append(value)

            corrupted.append(new_row)

        return corrupted

    # =========================
    # Map Fusion
    # =========================

    # Occupancy above which a cell counts as a wall (matches the localizer).
    _WALL = 0.6
    # Below which a cell counts as confidently free (matches the planner/frontier).
    _FREE = 0.4

    def _merge_maps(self, target_map, target_locked, incoming_map, incoming_locked,
                    protect=None):
        """
        Confident-knowledge merge: adopt only the cells the sender has *locked*.

        A cell locks (in the agent's own occupancy update) only after enough
        consistent direct observations to saturate past the free/wall thresholds,
        so a locked cell is the sender's confirmed, settled knowledge - not a
        noisy half-observation. We copy those faithfully into any cell we have not
        locked ourselves, and lock them (we now hold them as confident too). Cells
        the sender is still unsure about are not shared at all.

        This is what makes sharing safe. Earlier merges fused *unlocked*,
        mid-accumulation values - which is precisely where corruption came from:
        a transient low reading on a true wall would propagate and, gossiped
        around, get locked as free, leaving agents driving into real walls. By
        exchanging only settled cells (the same information a single shared map
        would hold), the merged map stays as clean as each agent's own sensing.
        """
        height = len(target_map)
        width = len(target_map[0])
        mask_changed = False

        for y in range(height):
            for x in range(width):
                if target_locked[y][x]:
                    # Contested WALL: we have a cell locked as a wall that a peer
                    # confidently sees as free. Rather than silently keep our wall,
                    # distrust it and re-sense (must re-accumulate before re-locking).
                    # Asymmetric on purpose - we only reconsider our own *walls* a
                    # peer contradicts (the phantom-wall case), never clear free
                    # space, so a frame-mismatched peer can't open real walls.
                    if (self.merge_reconsider and incoming_locked[y][x]
                            and target_map[y][x] > self._WALL
                            and incoming_map[y][x] <= self._WALL):
                        target_locked[y][x] = False
                        target_map[y][x] = 0.5
                        mask_changed = True  # a believed wall is being cleared
                    continue  # our own confident knowledge otherwise stands
                if incoming_locked[y][x]:
                    inc = incoming_map[y][x]
                    # Don't let a peer re-impose a WALL on a cell we recently eroded
                    # *and still see as free*: our fresh local evidence wins until it
                    # ages out. We require our own value to be free (< _FREE), so a
                    # bad erosion (a real wall we re-sense as wall) is NOT protected
                    # and the peer correctly restores it - only a confirmed-clear
                    # phantom is shielded from being re-locked.
                    if (protect and inc > self._WALL and (x, y) in protect
                            and target_map[y][x] < self._FREE):
                        continue
                    # Only a wall-threshold crossing matters to the localizer.
                    if (target_map[y][x] > self._WALL) != (inc > self._WALL):
                        mask_changed = True
                    target_map[y][x] = inc
                    target_locked[y][x] = True

        return mask_changed