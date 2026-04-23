"""
Geometry helper functions and command shorthand wrappers for the
auto builder DSL.

Context-manager containers (``sequence``, ``parallel``, ``race``) are
re-exported from ``auto_lib`` for convenience.
"""

from __future__ import annotations

from typing import Optional

from . import auto_action as AutoAction
from .units import Second

# Re-export context-manager containers so callers can do:
#   from .shorthands import sequence, parallel, race
from .auto_lib import parallel, race, sequence  # noqa: F401


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------


def translation2d(x: float = 0.0, y: float = 0.0) -> AutoAction.Translation2d:
    return AutoAction.Translation2d(x=x, y=y)


def rotation2d(degrees: float = 0.0) -> AutoAction.Rotation2d:
    return AutoAction.Rotation2d(degrees=degrees)


def pose2d(
    x: float = 0.0, y: float = 0.0, angle_degrees: float = 0.0
) -> AutoAction.Pose2d:
    return AutoAction.Pose2d(
        translation=translation2d(x, y),
        rotation=rotation2d(angle_degrees),
    )


def translation3d(
    x: float = 0.0, y: float = 0.0, z: float = 0.0
) -> AutoAction.Translation3d:
    return AutoAction.Translation3d(x=x, y=y, z=z)


def rotation3d(
    roll_degrees: float = 0.0,
    pitch_degrees: float = 0.0,
    yaw_degrees: float = 0.0,
) -> AutoAction.Rotation3d:
    return AutoAction.Rotation3d(
        roll=roll_degrees, pitch=pitch_degrees, yaw=yaw_degrees
    )


def pose3d(
    x: float = 0.0,
    y: float = 0.0,
    z: float = 0.0,
    roll_degrees: float = 0.0,
    pitch_degrees: float = 0.0,
    yaw_degrees: float = 0.0,
) -> AutoAction.Pose3d:
    return AutoAction.Pose3d(
        translation=translation3d(x, y, z),
        rotation=rotation3d(roll_degrees, pitch_degrees, yaw_degrees),
    )


def transform2d(
    translation: Optional[AutoAction.Translation2d] = None,
    rotation: Optional[AutoAction.Rotation2d] = None,
) -> AutoAction.Transform2d:
    return AutoAction.Transform2d(
        translation=translation if translation is not None else AutoAction.Translation2d(),
        rotation=rotation if rotation is not None else AutoAction.Rotation2d(),
    )


# ---------------------------------------------------------------------------
# Primitive action shorthands
# ---------------------------------------------------------------------------


def wait(seconds: float) -> None:
    """Insert a Wait action into the current context."""
    AutoAction.Wait(delay=Second.of(seconds)).add()


def print_msg(message: str) -> None:
    """Insert a Print action into the current context."""
    AutoAction.Print(message=message).add()


def autopilot(
    target: Optional[AutoAction.APTarget] = None,
    *,
    target_pose: Optional[AutoAction.Pose2d] = None,
    entry_angle: Optional[AutoAction.Rotation2d] = None,
    velocity: Optional[float] = None,
    rotation_radius: Optional[AutoAction.Measure] = None,
    constraints: Optional[AutoAction.APConstraints] = None,
    profile: Optional[AutoAction.APProfile] = None,
    pid_gains: Optional[AutoAction.PIDGains] = None,
    can_mirror: Optional[bool] = None,
) -> None:
    """Insert an AutoPilotAction into the current context.

    You can pass a pre-built ``APTarget`` as the first argument, or use the
    convenience keyword arguments ``target_pose``, ``entry_angle``,
    ``velocity``, and ``rotation_radius`` to build one inline.
    """
    if target is None:
        target = AutoAction.APTarget(
            reference=target_pose,
            entry_angle=entry_angle,
            velocity=velocity if velocity is not None else 0.0,
            rotation_radius=rotation_radius,
        )
    AutoAction.AutoPilotAction(
        target=target,
        constraints=constraints,
        profile=profile,
        pid_gains=pid_gains,
        can_mirror=can_mirror,
    ).add()


def x_based_autopilot(
    target: Optional[AutoAction.APTarget] = None,
    *,
    target_pose: Optional[AutoAction.Pose2d] = None,
    entry_angle: Optional[AutoAction.Rotation2d] = None,
    velocity: Optional[float] = None,
    rotation_radius: Optional[AutoAction.Measure] = None,
    constraints: Optional[AutoAction.APConstraints] = None,
    profile: Optional[AutoAction.APProfile] = None,
    pid_gains: Optional[AutoAction.PIDGains] = None,
    can_mirror: Optional[bool] = None,
) -> None:
    """Insert an XBasedAutoPilotAction into the current context."""
    if target is None:
        target = AutoAction.APTarget(
            reference=target_pose,
            entry_angle=entry_angle,
            velocity=velocity if velocity is not None else 0.0,
            rotation_radius=rotation_radius,
        )
    AutoAction.XBasedAutoPilotAction(
        target=target,
        constraints=constraints,
        profile=profile,
        pid_gains=pid_gains,
        can_mirror=can_mirror,
    ).add()


# Keep the old name as an alias for backwards compat
x_based_autopilot_action = x_based_autopilot


def stop_drive() -> None:
    """Insert a StopDriveAction into the current context."""
    AutoAction.StopDriveAction().add()


def deploy_intake() -> None:
    """Insert a DeployIntakeAction into the current context."""
    AutoAction.DeployIntakeAction().add()


def stow_intake() -> None:
    """Insert a StowIntakeAction into the current context."""
    AutoAction.StowIntakeAction().add()


def climb_search() -> None:
    """Insert a ClimbSearchAction into the current context."""
    AutoAction.ClimbSearchAction().add()


def climb_hang() -> None:
    """Insert a ClimbHangAction into the current context."""
    AutoAction.ClimbHangAction().add()


def reference(auto_name: str) -> None:
    """Insert an AutoReference into the current context."""
    AutoAction.AutoReference(name=auto_name).add()

def startShooting() -> None:
    """Insert a StartShootingAction into the current context."""
    AutoAction.StartShooting().add()

def stopShooting() -> None:
    """Insert a StopShootingAction into the current context."""
    AutoAction.StopShooting().add()

def followPath(path_name, mirror_path=None, can_mirror=None):
    AutoAction.FollowPathPlannerPath(path_name=path_name, mirror_path=mirror_path, can_mirror=can_mirror).add()

def followBLinePath(path_name):
    AutoAction.FollowBLinePath(path_name=path_name).add()
