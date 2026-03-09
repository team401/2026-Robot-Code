"""
Auto registration system with a context-manager DSL for building
nested command trees (Sequence, Parallel, Race).

Usage::

    @auto("My Auto")
    def my_auto():
        with sequence():
            autopilot(target_pose=...)
            with parallel():
                wait(1.0)
                climb_search()
"""

from __future__ import annotations

import json
from contextlib import contextmanager
from typing import Any, Callable, Dict, List, Optional

from . import auto_action as AutoAction
from .auto_action import set_add_command_hook

# ---------------------------------------------------------------------------
# Internal pointer stack
# ---------------------------------------------------------------------------

_command_pointers: List[List[Any]] = []


def _current_pointer() -> Optional[List[Any]]:
    return _command_pointers[-1] if _command_pointers else None


def _push_pointer(pointer: List[Any]) -> None:
    _command_pointers.append(pointer)


def _pop_pointer() -> None:
    if _command_pointers:
        _command_pointers.pop()


def _add_command(command: Any) -> None:
    ptr = _current_pointer()
    if ptr is not None:
        ptr.append(command)


# Wire up the hook so that .add() on any AutoAction instance calls _add_command.
set_add_command_hook(_add_command)

# ---------------------------------------------------------------------------
# Context-manager containers
# ---------------------------------------------------------------------------


@contextmanager
def sequence():
    """Context manager that groups all commands inside into a Sequence."""
    container = AutoAction.Sequence()
    _push_pointer(container.actions)
    try:
        yield container
    finally:
        _pop_pointer()
    _add_command(container)


@contextmanager
def parallel():
    """Context manager that groups all commands inside into a Parallel."""
    container = AutoAction.Parallel()
    _push_pointer(container.actions)
    try:
        yield container
    finally:
        _pop_pointer()
    _add_command(container)


@contextmanager
def race():
    """Context manager that groups all commands inside into a Race."""
    container = AutoAction.Race()
    _push_pointer(container.actions)
    try:
        yield container
    finally:
        _pop_pointer()
    _add_command(container)


# ---------------------------------------------------------------------------
# Auto registration
# ---------------------------------------------------------------------------

_autos: Dict[str, Any] = {}


def auto(name: str):
    """
    Decorator that defines and registers a named autonomous routine.

    Usage::

        @auto("My Auto")
        def my_auto():
            autopilot(target_pose=...)
            wait(1.0)
    """
    def decorator(build: Callable[[], None]):
        global _command_pointers
        _command_pointers = []
        root = AutoAction.Sequence()
        _push_pointer(root.actions)

        try:
            build()
        finally:
            _command_pointers = []

        _autos[name] = root
        return build
    return decorator


def get_auto(name: str) -> Optional[Any]:
    return _autos.get(name)


def get_autos() -> Dict[str, Any]:
    return _autos


# ---------------------------------------------------------------------------
# Serialization
# ---------------------------------------------------------------------------

def _serialize_value(obj: Any) -> Any:
    """Recursively convert objects to JSON-serializable dicts."""
    if obj is None:
        return None
    if hasattr(obj, "to_dict"):
        return obj.to_dict()
    if isinstance(obj, list):
        return [_serialize_value(item) for item in obj]
    if isinstance(obj, dict):
        return {k: _serialize_value(v) for k, v in obj.items()}
    return obj


def serialize_autos() -> str:
    """Serializes all registered autos to a JSON string."""
    result = {name: _serialize_value(action) for name, action in _autos.items()}
    return json.dumps(result, indent=4)
