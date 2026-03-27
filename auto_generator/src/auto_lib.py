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
_routines: Dict[str, Any] = {}


def auto(name: str, can_be_mirrored: bool = True, should_be_flipped: bool = True):
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
        auto = AutoAction.Auto(root_action=root, can_be_mirrored=can_be_mirrored, should_be_flipped=should_be_flipped)

        _push_pointer(root.actions)

        try:
            build()
        finally:
            _command_pointers = []

        _autos[name] = auto
        return build
    return decorator


# ---------------------------------------------------------------------------
# Routine registration (reusable subroutines referenced by name)
# ---------------------------------------------------------------------------

class _Routines:
    """
    Registry of named routines.

    Accessing ``routines.<name>()`` emits an AutoReference into the current
    context (compact, no inlining).  Calling the decorated function directly
    inlines the body and prints a warning.
    """

    _registry: Dict[str, Callable[[], None]] = {}

    def add(self, name: str, func: Callable[[], None]) -> None:
        self._registry[name] = func

    def __getattr__(self, name: str) -> Callable[[], None]:
        if name.startswith("_"):
            raise AttributeError(name)
        if name in self._registry:
            def _emit_reference() -> None:
                AutoAction.AutoReference(name=name).add()
            return _emit_reference
        raise AttributeError(f"Unknown routine '{name}'. Registered: {list(self._registry.keys())}")


routines = _Routines()


def routine(func_or_name):
    """
    Decorator that registers a function as a named routine.

    Can be used with or without an explicit name::

        @routine
        def climb_left():
            ...

        @routine("ClimbRight")
        def _climb_right():
            ...

    The routine body is built eagerly and stored in ``_routines`` so it
    appears under the "routines" key in Autos.json.  Use ``routines.<name>()``
    inside an auto to emit an AutoReference instead of inlining.
    """

    def _register(name: str, build: Callable[[], None]) -> Callable[[], None]:
        # Build and register the routine body
        global _command_pointers
        _command_pointers = []
        root = AutoAction.Sequence()
        _push_pointer(root.actions)
        try:
            build()
        finally:
            _command_pointers = []
        _routines[name] = root

        routines.add(name, build)

        # The returned wrapper inlines the body with a warning
        def inner(ignore_warning=False) -> None:
            if not ignore_warning:
                print(
                    f"WARNING: routine method {name} was called directly. "
                    f"This results in the routine being inlined into the auto, "
                    f"which may result in large file sizes. To rectify the issue, "
                    f"replace the call with `routines.{name}()`"
            )
            build()

        return inner

    if callable(func_or_name):
        # @routine  (no parentheses — use the function name)
        return _register(func_or_name.__name__, func_or_name)
    else:
        # @routine("CustomName")
        def decorator(func: Callable[[], None]) -> Callable[[], None]:
            return _register(func_or_name, func)
        return decorator


def get_auto(name: str) -> Optional[Any]:
    return _autos.get(name)


def get_autos() -> Dict[str, Any]:
    return _autos


def get_routine(name: str) -> Optional[Any]:
    return _routines.get(name)


def get_routines() -> Dict[str, Any]:
    return _routines


# ---------------------------------------------------------------------------
# Serialization
# ---------------------------------------------------------------------------

def _serialize_value(obj: Any) -> Any:
    """Recursively convert objects to JSON-serializable dicts."""
    if obj is None:
        return None
    if hasattr(obj, "to_dict"):
        return _serialize_value(obj.to_dict())
    if isinstance(obj, list):
        return [_serialize_value(item) for item in obj]
    if isinstance(obj, dict):
        return {k: _serialize_value(v) for k, v in obj.items()}
    return obj


def serialize_autos() -> str:
    """Serializes all registered autos to a JSON string."""
    result = {name: _serialize_value(action) for name, action in _autos.items()}
    return json.dumps(result, indent=4)
