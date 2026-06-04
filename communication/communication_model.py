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
        rng_manager=None,
    ):
        self.mode = mode
        self.communication_range = communication_range

        self.packet_loss_rate = packet_loss_rate
        self.corruption_rate = corruption_rate

        self.rng_manager = rng_manager
        self.rng = rng_manager.behaviour_rng() if rng_manager else None

    # =========================
    # Main API
    # =========================

    def communicate(self, agent, agents: List):
        """
        Share information between agents.
        """

        for other in agents:
            if other.id == agent.id:
                continue

            if not self._can_communicate(agent, other):
                continue

            # Packet loss check
            if self._packet_lost():
                continue

            # Send map
            received_map = self._transmit_map(other.internal_map)

            # Merge maps
            self._merge_maps(agent.internal_map, received_map)

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

    def _merge_maps(self, target_map, incoming_map):
        """
        Merge maps using averaging.
        """

        height = len(target_map)
        width = len(target_map[0])

        for y in range(height):
            for x in range(width):
                target_map[y][x] = (target_map[y][x] + incoming_map[y][x]) / 2