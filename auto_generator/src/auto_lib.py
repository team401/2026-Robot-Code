"""
Python equivalent of AutoLib.ts.

Provides the auto registration system with a pointer-stack DSL for building
nested command trees (Sequence, Parallel, Race).
"""

from __future__ import annotations

import json
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
# Public API
# ---------------------------------------------------------------------------


def get_last_command() -> Optional[Any]:
    """Returns the last command added in the current context, or None."""
    ptr = _current_pointer()
    if ptr:
        return ptr[-1]
    return None


def with_container(
    container: AutoAction.Sequence | AutoAction.Parallel | AutoAction.Race,
    build: Callable[[], None],
) -> None:
    """
    Runs ``build`` with a container command as the active context, then adds the
    finished container to the parent context.

    Example::

        with_container(AutoAction.Parallel(), lambda: (
            drive(target_pose=...).add(),
            shoot().add(),
        ))
    """
    _push_pointer(container.actions)
    try:
        build()
    finally:
        _pop_pointer()
    _add_command(container)


# ---------------------------------------------------------------------------
# Auto registration
# ---------------------------------------------------------------------------

_autos: Dict[str, Any] = {}


def auto(name: str, build: Callable[[], None]) -> None:
    """
    Defines and registers a named autonomous routine.

    The ``build`` callback runs synchronously; commands created inside it are
    appended to a top-level Sequence via the pointer stack.
    """
    global _command_pointers
    _command_pointers = []
    root = AutoAction.Sequence()
    _push_pointer(root.actions)

    try:
        build()
    finally:
        _command_pointers = []

    _autos[name] = root


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
