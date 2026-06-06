"""
sensor_model.py

Field-of-view sensor with line-of-sight occlusion.

The agent observes cells within sensor_range, but cannot see through walls or
around corners: walls cast shadows that hide everything behind them. Visibility
is computed with recursive shadowcasting (the standard roguelike FOV algorithm)
across 8 octants, so the agent sees the wall faces in front of it but nothing
beyond.
"""

from typing import List, Tuple

from sensing.fov import visible_cells

Position = Tuple[float, float]
Observation = Tuple[int, int, bool]


class SensorModel:
    def __init__(
        self,
        sensor_range: float,
        mode: str = "radius",
        false_positive_rate: float = 0.0,
        false_negative_rate: float = 0.0,
        rng_manager=None,
    ):
        self.sensor_range = sensor_range
        self.mode = mode

        self.false_positive_rate = false_positive_rate
        self.false_negative_rate = false_negative_rate

        self.rng = rng_manager.behaviour_rng() if rng_manager else None

    # =========================
    # Main API
    # =========================

    def sense(self, environment, position: Position) -> List[Observation]:
        """
        Return observations for every cell currently visible to the agent.
        Cells hidden behind walls (shadowed) are not reported.
        """
        cx, cy = int(position[0]), int(position[1])
        radius = int(self.sensor_range)

        observations = []
        for x, y in self._visible_cells(environment, cx, cy, radius):
            true_occ = not environment.is_free(x, y)
            observations.append((x, y, self._noise(true_occ)))

        return observations

    # =========================
    # Field of view
    # =========================

    def _visible_cells(self, environment, cx, cy, radius) -> set:
        """Cells visible from (cx, cy) within radius, occluded by walls."""
        return visible_cells(
            is_blocked=lambda x, y: not environment.is_free(x, y),
            in_bounds=environment.map.in_bounds,
            cx=cx, cy=cy, radius=radius,
        )

    # =========================
    # Noise
    # =========================

    def _noise(self, truth: bool) -> bool:
        if self.rng is None:
            return truth

        if truth:
            if self.rng.random() < self.false_negative_rate:
                return False
        else:
            if self.rng.random() < self.false_positive_rate:
                return True

        return truth
