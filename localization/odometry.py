"""
odometry.py

Implements odometry-based localization with Gaussian drift.

Characteristics:
- Tracks motion via velocity integration
- Accumulates error over time
- No correction from environment (pure dead reckoning)
"""

from typing import Tuple

Position = Tuple[float, float]


class OdometryLocalization:
    def __init__(
        self,
        noise_sigma: float = 0.1,
        rng_manager=None,
    ):
        """
        noise_sigma:
            Standard deviation of Gaussian noise applied per update
        """
        self.noise_sigma = noise_sigma

        self.rng_manager = rng_manager
        self.rng = rng_manager.behaviour_rng() if rng_manager else None

    # =========================
    # Main API
    # =========================

    def update(
        self,
        believed_position: Position,
        velocity: Position,
        observations=None,
        internal_map=None,
        anchors=None,
        map_version=None,
    ) -> Position:
        """
        Update believed position using odometry + noise.

        `anchors` (inter-agent pose hints) and `map_version` are accepted for a
        uniform localizer interface but ignored - dead reckoning has no
        correction step.
        """

        
        dx, dy = velocity  # now this is actual motion

        new_x = believed_position[0] + dx
        new_y = believed_position[1] + dy


        # Apply Gaussian noise
        if self.rng is not None and self.noise_sigma > 0:
            noise_x = self.rng.gauss(0, self.noise_sigma)
            noise_y = self.rng.gauss(0, self.noise_sigma)

            new_x += noise_x
            new_y += noise_y

        return (new_x, new_y)