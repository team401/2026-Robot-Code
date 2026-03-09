"""
Python equivalent of the auto-generated AutoAction.ts.

Every class serializes to the same JSON structure the Java robot code expects.
The `add()` hook system is replicated so the auto builder DSL works identically.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from typing import Callable, List, Optional, Any

from . import units as Units


# ---------------------------------------------------------------------------
# Global add-command hook (mirrors setAddCommandHook / _addCommandHook)
# ---------------------------------------------------------------------------

_add_command_hook: Optional[Callable[[Any], None]] = None


def set_add_command_hook(hook: Callable[[Any], None]) -> None:
    global _add_command_hook
    _add_command_hook = hook


def _call_hook(obj: Any) -> None:
    if _add_command_hook is not None:
        _add_command_hook(obj)


# ---------------------------------------------------------------------------
# Geometry classes
# ---------------------------------------------------------------------------

@dataclass
class Rotation2d:
    degrees: float = 0.0

    def to_dict(self) -> dict:
        return {"degrees": self.degrees}


@dataclass
class Translation2d:
    x: float = 0.0
    y: float = 0.0

    def to_dict(self) -> dict:
        return {"x": self.x, "y": self.y}


@dataclass
class Pose2d:
    translation: Optional[Translation2d] = None
    rotation: Optional[Rotation2d] = None

    def __post_init__(self):
        if self.translation is None:
            self.translation = Translation2d()
        if self.rotation is None:
            self.rotation = Rotation2d()

    def to_dict(self) -> dict:
        return {
            "translation": self.translation.to_dict() if self.translation else Translation2d().to_dict(),
            "rotation": self.rotation.to_dict() if self.rotation else Rotation2d().to_dict(),
        }


@dataclass
class Transform2d:
    translation: Optional[Translation2d] = None
    rotation: Optional[Rotation2d] = None

    def __post_init__(self):
        if self.translation is None:
            self.translation = Translation2d()
        if self.rotation is None:
            self.rotation = Rotation2d()

    def to_dict(self) -> dict:
        return {
            "translation": self.translation.to_dict() if self.translation else Translation2d().to_dict(),
            "rotation": self.rotation.to_dict() if self.rotation else Rotation2d().to_dict(),
        }


@dataclass
class Rotation3d:
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0

    def to_dict(self) -> dict:
        return {"roll": self.roll, "pitch": self.pitch, "yaw": self.yaw}


@dataclass
class Translation3d:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    def to_dict(self) -> dict:
        return {"x": self.x, "y": self.y, "z": self.z}


@dataclass
class Transform3d:
    translation: Optional[Translation3d] = None
    rotation: Optional[Rotation3d] = None

    def __post_init__(self):
        if self.translation is None:
            self.translation = Translation3d()
        if self.rotation is None:
            self.rotation = Rotation3d()

    def to_dict(self) -> dict:
        return {
            "translation": self.translation.to_dict() if self.translation else Translation3d().to_dict(),
            "rotation": self.rotation.to_dict() if self.rotation else Rotation3d().to_dict(),
        }


@dataclass
class Pose3d:
    translation: Optional[Translation3d] = None
    rotation: Optional[Rotation3d] = None

    def __post_init__(self):
        if self.translation is None:
            self.translation = Translation3d()
        if self.rotation is None:
            self.rotation = Rotation3d()

    def to_dict(self) -> dict:
        return {
            "translation": self.translation.to_dict() if self.translation else Translation3d().to_dict(),
            "rotation": self.rotation.to_dict() if self.rotation else Rotation3d().to_dict(),
        }


# ---------------------------------------------------------------------------
# AutoPilot helper classes
# ---------------------------------------------------------------------------

@dataclass
class APTarget:
    reference: Optional[Pose2d] = None
    entry_angle: Optional[Rotation2d] = None
    velocity: float = 0.0
    rotation_radius: Optional[Units.Measure] = None

    def __post_init__(self):
        if self.reference is None:
            self.reference = Pose2d()

    def to_dict(self) -> dict:
        d: dict = {
            "reference": self.reference.to_dict() if self.reference else Pose2d().to_dict(),
            "velocity": self.velocity,
        }
        if self.entry_angle is not None:
            d["entryAngle"] = self.entry_angle.to_dict()
        if self.rotation_radius is not None:
            d["rotationRadius"] = self.rotation_radius.to_dict()
        return d


@dataclass
class APConstraints:
    velocity: float = 0.0
    acceleration: float = 0.0
    jerk: float = 0.0

    def to_dict(self) -> dict:
        return {
            "velocity": self.velocity,
            "acceleration": self.acceleration,
            "jerk": self.jerk,
        }


@dataclass
class APProfile:
    constraints: Optional[APConstraints] = None
    error_xy: Optional[Units.Measure] = None
    error_theta: Optional[Units.Measure] = None
    beeline_radius: Optional[Units.Measure] = None

    def __post_init__(self):
        if self.constraints is None:
            self.constraints = APConstraints()

    def to_dict(self) -> dict:
        d: dict = {
            "constraints": self.constraints.to_dict() if self.constraints else APConstraints().to_dict(),
        }
        if self.error_xy is not None:
            d["errorXY"] = self.error_xy.to_dict()
        if self.error_theta is not None:
            d["errorTheta"] = self.error_theta.to_dict()
        if self.beeline_radius is not None:
            d["beelineRadius"] = self.beeline_radius.to_dict()
        return d


@dataclass
class PIDGains:
    kP: float = 0.0
    kI: float = 0.0
    kD: float = 0.0
    kS: float = 0.0
    kG: float = 0.0
    kV: float = 0.0
    kA: float = 0.0

    def to_dict(self) -> dict:
        return {
            "kP": self.kP,
            "kI": self.kI,
            "kD": self.kD,
            "kS": self.kS,
            "kG": self.kG,
            "kV": self.kV,
            "kA": self.kA,
        }


# ---------------------------------------------------------------------------
# Auto action classes (commands)
# ---------------------------------------------------------------------------

@dataclass
class AutoReference:
    type: str = field(default="AutoReference", init=False)
    name: str = ""

    def add(self):
        _call_hook(self)
        return self

    def to_dict(self) -> dict:
        return {"type": self.type, "name": self.name}


@dataclass
class Deadline:
    type: str = field(default="Deadline", init=False)
    deadline: Any = None
    others: List[Any] = field(default_factory=list)

    def add(self):
        _call_hook(self)
        return self

    def to_dict(self) -> dict:
        return {
            "type": self.type,
            "deadline": _to_dict(self.deadline),
            "others": [_to_dict(a) for a in self.others],
        }


@dataclass
class Sequence:
    type: str = field(default="Sequence", init=False)
    actions: List[Any] = field(default_factory=list)

    def add(self):
        _call_hook(self)
        return self

    def to_dict(self) -> dict:
        return {
            "type": self.type,
            "actions": [_to_dict(a) for a in self.actions],
        }


@dataclass
class Parallel:
    type: str = field(default="Parallel", init=False)
    actions: List[Any] = field(default_factory=list)

    def add(self):
        _call_hook(self)
        return self

    def to_dict(self) -> dict:
        return {
            "type": self.type,
            "actions": [_to_dict(a) for a in self.actions],
        }


@dataclass
class Race:
    type: str = field(default="Race", init=False)
    actions: List[Any] = field(default_factory=list)

    def add(self):
        _call_hook(self)
        return self

    def to_dict(self) -> dict:
        return {
            "type": self.type,
            "actions": [_to_dict(a) for a in self.actions],
        }


@dataclass
class Wait:
    type: str = field(default="Wait", init=False)
    delay: Optional[Units.Measure] = None

    def add(self):
        _call_hook(self)
        return self

    def to_dict(self) -> dict:
        d: dict = {"type": self.type}
        if self.delay is not None:
            d["delay"] = self.delay.to_dict()
        return d


@dataclass
class Print:
    type: str = field(default="Print", init=False)
    message: str = ""

    def add(self):
        _call_hook(self)
        return self

    def to_dict(self) -> dict:
        return {"type": self.type, "message": self.message}


@dataclass
class AutoPilotAction:
    type: str = field(default="AutoPilotAction", init=False)
    target: Optional[APTarget] = None
    profile: Optional[APProfile] = None
    constraints: Optional[APConstraints] = None
    pid_gains: Optional[PIDGains] = None
    alliance_relative: Optional[bool] = None

    def __post_init__(self):
        if self.target is None:
            self.target = APTarget()

    def add(self):
        _call_hook(self)
        return self

    def to_dict(self) -> dict:
        d: dict = {
            "type": self.type,
            "target": self.target.to_dict() if self.target else APTarget().to_dict(),
        }
        if self.profile is not None:
            d["profile"] = self.profile.to_dict()
        if self.constraints is not None:
            d["constraints"] = self.constraints.to_dict()
        if self.pid_gains is not None:
            d["pidGains"] = self.pid_gains.to_dict()
        if self.alliance_relative is not None:
            d["allianceRelative"] = self.alliance_relative
        return d


@dataclass
class XBasedAutoPilotAction:
    type: str = field(default="XBasedAutoPilotAction", init=False)
    target: Optional[APTarget] = None
    profile: Optional[APProfile] = None
    constraints: Optional[APConstraints] = None
    pid_gains: Optional[PIDGains] = None
    alliance_relative: Optional[bool] = None

    def __post_init__(self):
        if self.target is None:
            self.target = APTarget()

    def add(self):
        _call_hook(self)
        return self

    def to_dict(self) -> dict:
        d: dict = {
            "type": self.type,
            "target": self.target.to_dict() if self.target else APTarget().to_dict(),
        }
        if self.profile is not None:
            d["profile"] = self.profile.to_dict()
        if self.constraints is not None:
            d["constraints"] = self.constraints.to_dict()
        if self.pid_gains is not None:
            d["pidGains"] = self.pid_gains.to_dict()
        if self.alliance_relative is not None:
            d["allianceRelative"] = self.alliance_relative
        return d


@dataclass
class StopDriveAction:
    type: str = field(default="StopDriveAction", init=False)

    def add(self):
        _call_hook(self)
        return self

    def to_dict(self) -> dict:
        return {"type": self.type}


@dataclass
class DeployIntakeAction:
    type: str = field(default="DeployIntakeAction", init=False)

    def add(self):
        _call_hook(self)
        return self

    def to_dict(self) -> dict:
        return {"type": self.type}


@dataclass
class StowIntakeAction:
    type: str = field(default="StowIntakeAction", init=False)

    def add(self):
        _call_hook(self)
        return self

    def to_dict(self) -> dict:
        return {"type": self.type}


@dataclass
class ClimbSearchAction:
    type: str = field(default="ClimbSearchAction", init=False)

    def add(self):
        _call_hook(self)
        return self

    def to_dict(self) -> dict:
        return {"type": self.type}


@dataclass
class ClimbHangAction:
    type: str = field(default="ClimbHangAction", init=False)

    def add(self):
        _call_hook(self)
        return self

    def to_dict(self) -> dict:
        return {"type": self.type}


# ---------------------------------------------------------------------------
# Serialization helper
# ---------------------------------------------------------------------------

def _to_dict(obj: Any) -> Any:
    """Recursively convert an object to a JSON-serializable dict."""
    if obj is None:
        return None
    if hasattr(obj, "to_dict"):
        return obj.to_dict()
    return obj
