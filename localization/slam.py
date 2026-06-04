"""
slam.py

Simplified SLAM-like localization.

Combines:
- Odometry (with drift)
- Sensor-based correction

This is NOT full SLAM, but provides:
- Drift correction
- Map alignment behaviour
"""

from typing import Tuple, List
import math

Position = Tuple[float, float]
Observation = Tuple[int, int, bool]


class SLAMLocalization:
    def __init__(
        self,
        noise_sigma: float = 0.1,
        correction_strength: float = 0.1,
        rng_manager=None,
    ):
        """
        noise_sigma:
            Odometry drift

        correction_strength:
            How strongly sensor observations pull the position
        """
        self.noise_sigma = noise_sigma
        self.correction_strength = correction_strength

        self.rng_manager = rng_manager
        self.rng = rng_manager.behaviour_rng() if rng_manager else None

    # =========================
    # Main API
    # =========================

    def update(
        self,
        believed_position: Position,
        velocity: Position,
        observations: List[Observation],
    ) -> Position:
        """
        SLAM update:
        1. Apply odometry
        2. Apply correction from observations
        """

        # -------------------------
        # 1. Odometry Step
        # -------------------------
        vx, vy = velocity

        new_x = believed_position[0] + vx
        new_y = believed_position[1] + vy

        if self.rng is not None and self.noise_sigma > 0:
            new_x += self.rng.gauss(0, self.noise_sigma)
            new_y += self.rng.gauss(0, self.noise_sigma)

        estimated_pos = (new_x, new_y)

        # -------------------------
        # 2. Sensor Correction
        # -------------------------
        if not observations:
            return estimated_pos

        correction_dx = 0.0
        correction_dy = 0.0
        count = 0

        px, py = int(estimated_pos[0]), int(estimated_pos[1])

        for obs_x, obs_y, occupied in observations:

            # Only use nearby observations for correction
            dx = obs_x - px
            dy = obs_y - py
            dist = math.hypot(dx, dy)

            if dist > 3:  # local correction radius
                continue

            # Compute direction from estimate to observation
            direction_x = dx
            direction_y = dy

            # Normalize
            mag = math.hypot(direction_x, direction_y)
            if mag == 0:
                continue

            direction_x /= mag
            direction_y /= mag

            # Apply correction bias:
            # If we see a wall, pull toward it slightly (anchor)
            # If free, push away slightly (avoid crowding)
            if occupied:
                weight = 1.0
            else:
                weight = -0.5

            correction_dx += direction_x * weight
            correction_dy += direction_y * weight
            count += 1

        if count > 0:
            correction_dx /= count
            correction_dy /= count

            new_x += correction_dx * self.correction_strength
            new_y += correction_dy * self.correction_strength

        return (new_x, new_y)