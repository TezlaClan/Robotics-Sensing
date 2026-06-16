"""
exact.py

Ground-truth localization baseline.

Integrates the agent's actual motion with no noise, so the believed position
exactly tracks the true position. Useful for comparing against odometry/SLAM
to isolate the effect of localization error.
"""

from typing import Tuple, List

Position = Tuple[float, float]
Observation = Tuple[int, int, bool]


class ExactLocalization:
    def __init__(self, rng_manager=None):
        # No state / noise; signature kept uniform with the other localizers.
        self.rng_manager = rng_manager

    def update(
        self,
        believed_position: Position,
        motion: Position,
        observations: List[Observation] = None,
        internal_map=None,
        anchors=None,
        map_version=None,
        locked=None,
        anchor_mode="world",
    ) -> Position:
        return (
            believed_position[0] + motion[0],
            believed_position[1] + motion[1],
        )
