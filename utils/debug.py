"""
debug.py

Lightweight global switch for verbose console output.

Set once at startup from config["debug"]; use dprint() for any trace/debug
message that should be silenced when debug is off. The periodic step counter
and error messages use plain print() so they always show.
"""

_DEBUG = False


def set_debug(enabled: bool):
    """Enable or disable verbose debug output globally."""
    global _DEBUG
    _DEBUG = bool(enabled)


def is_debug() -> bool:
    return _DEBUG


def dprint(*args, **kwargs):
    """print() that only fires when debug output is enabled."""
    if _DEBUG:
        print(*args, **kwargs)
