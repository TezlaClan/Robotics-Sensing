"""
random_manager.py

Provides deterministic random number generators for:
- Map generation
- Agent behaviour

ALL randomness in the project MUST come from here.
"""

import random
from typing import Optional


class RandomManager:
    def __init__(self, map_seed: Optional[int] = None, behaviour_seed: Optional[int] = None):
        """
        Initialize deterministic RNGs.

        If seeds are None, random seeds are generated.
        """

        # Generate seeds if not provided
        if map_seed is None:
            map_seed = random.randint(0, 2**31 - 1)

        if behaviour_seed is None:
            behaviour_seed = random.randint(0, 2**31 - 1)

        self.map_seed = map_seed
        self.behaviour_seed = behaviour_seed

        # Create isolated RNGs
        self._map_rng = random.Random(self.map_seed)
        self._behaviour_rng = random.Random(self.behaviour_seed)

    # =========================
    # Accessors
    # =========================

    def map_rng(self) -> random.Random:
        """RNG for map generation"""
        return self._map_rng

    def behaviour_rng(self) -> random.Random:
        """RNG for agent behaviour, noise, communication"""
        return self._behaviour_rng

    # =========================
    # Utility passthroughs
    # =========================

    # Map RNG shortcuts
    def map_randint(self, a: int, b: int) -> int:
        return self._map_rng.randint(a, b)

    def map_random(self) -> float:
        return self._map_rng.random()

    def map_choice(self, seq):
        return self._map_rng.choice(seq)

    def map_shuffle(self, seq):
        self._map_rng.shuffle(seq)

    # Behaviour RNG shortcuts
    def behaviour_randint(self, a: int, b: int) -> int:
        return self._behaviour_rng.randint(a, b)

    def behaviour_random(self) -> float:
        return self._behaviour_rng.random()

    def behaviour_choice(self, seq):
        return self._behaviour_rng.choice(seq)

    def behaviour_gauss(self, mu: float, sigma: float) -> float:
        return self._behaviour_rng.gauss(mu, sigma)

    def behaviour_shuffle(self, seq):
        self._behaviour_rng.shuffle(seq)

    # =========================
    # Debug / Logging
    # =========================

    def get_seeds(self):
        return {
            "map_seed": self.map_seed,
            "behaviour_seed": self.behaviour_seed
        }

    def __repr__(self):
        return f"RandomManager(map_seed={self.map_seed}, behaviour_seed={self.behaviour_seed})"