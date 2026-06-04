import math
from typing import List, Tuple

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

    def sense(self, environment, position: Position) -> List[Observation]:
        if self.mode == "radius":
            return self._sense_radius(environment, position)
        elif self.mode == "los":
            return self._sense_los(environment, position)
        else:
            raise ValueError("Invalid sensor mode")

    # =========================
    # Radius
    # =========================

    def _sense_radius(self, environment, position: Position) -> List: 
      px, py = int(position[0]), int(position[1])

      observations = []
      r = int(self.sensor_range)

      for dy in range(-r, r + 1):
          for dx in range(-r, r + 1):
              x = px + dx
              y = py + dy

              if not environment.map.in_bounds(x, y):
                  continue

              if math.hypot(dx, dy) > self.sensor_range:
                  continue

              true_occ = not environment.is_free(x, y)
              obs = self._noise(true_occ)

              observations.append((x, y, obs))

      return observations

    # =========================
    # LOS
    # =========================

    def _sense_los(self, environment, position: Position) -> List:
      py = int(position[0]), int(position[1])
      observations = []

      directions = [
          (1,0),(-1,0),(0,1),(0,-1),
          (1,1),(-1,-1),(1,-1),(-1,1)
      ]

      r = int(self.sensor_range)

      for dx, dy in directions:
          for step in range(1, r + 1):
              x = px + dx * step
              y = py + dy * step

              if not environment.map.in_bounds(x, y):
                  break

              true_occ = not environment.is_free(x, y)
              obs = self._noise(true_occ)

              observations.append((x, y, obs))

              if true_occ:
                  break

      return observations

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