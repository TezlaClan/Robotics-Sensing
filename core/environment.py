"""
environment.py

Provides environment interface used by agents.
Wraps the Map object.
"""

class Environment:
    def __init__(self, map_obj):
        self.map = map_obj

    def is_free(self, x: int, y: int) -> bool:
        return self.map.is_free(x, y)