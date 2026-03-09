"""
Geometry helper functions and command shorthand wrappers for the
auto builder DSL.

Context-manager containers (``sequence``, ``parallel``, ``race``) are
re-exported from ``auto_lib`` for convenience.
"""

from __future__ import annotations

from typing import Optional

from . import auto_action as AutoAction

# Re-export context-manager containers so callers can do:
#   from .shorthands import sequence, parallel, race
from .auto_lib import parallel, race, sequence  # noqa: F401


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------


def translation2d(x: float = 0.0, y: float = 0.0) -> AutoAction.Translation2d:
    return AutoAction.Translation2d(x=x, y=y)


def rotation2d(angle_degrees: float = 0.0) -> AutoAction.Rotation2d:
    return AutoAction.Rotation2d(degrees=angle_degrees)


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


def translation3d_to_pose2d(
    x: float = 0.0, y: float = 0.0, z: float = 0.0
) -> AutoAction.Pose2d:
    return AutoAction.Pose2d(
        translation=translation2d(x, y),
        rotation=rotation2d(),
    )


def translation2d_to_pose2d(
    t: AutoAction.Translation2d,
) -> AutoAction.Pose2d:
    return t.to_pose2d()


def pose2d_to_pose3d(
    p: AutoAction.Pose2d, z: float = 0.0
) -> AutoAction.Pose3d:
    return p.to_pose3d(z)


def pose3d_to_pose2d(p: AutoAction.Pose3d) -> AutoAction.Pose2d:
    return p.to_pose2d()


def translate2d_pose(
    p: AutoAction.Pose2d, t: AutoAction.Translation2d
) -> AutoAction.Pose2d:
    return p.translate_by(t)


def transform2d_pose(
    p: AutoAction.Pose2d, transform: AutoAction.Transform2d
) -> AutoAction.Pose2d:
    return p.transform_by(transform)


# ---------------------------------------------------------------------------
# Primitive action shorthands
# ---------------------------------------------------------------------------


def wait(seconds: float) -> None:
    """Insert a Wait action into the current context."""
    AutoAction.Wait(delay=AutoAction.Second.of(seconds)).add()


def print_msg(message: str) -> None:
    """Insert a Print action into the current context."""
    AutoAction.Print(message=message).add()


def autopilot(
    target: Optional[AutoAction.APTarget] = None,
    *,
    target_pose: Optional[AutoAction.Pose2d] = None,
    entry_angle: Optional[AutoAction.Rotation2d] = None,
    velocity: Optional[float] = None,
    constraints: Optional[AutoAction.APConstraints] = None,
    profile: Optional[AutoAction.APProfile] = None,
    pid_gains: Optional[AutoAction.PIDGains] = None,
    alliance_relative: Optional[bool] = None,
) -> None:
    """Insert an AutoPilotAction into the current context.

    You can pass a pre-built ``APTarget`` as the first argument, or use the
    convenience keyword arguments ``target_pose``, ``entry_angle``, and
    ``velocity`` to build one inline.
    """
    if target is None:
        target = AutoAction.APTarget(
            reference=target_pose,
            entry_angle=entry_angle,
            velocity=velocity if velocity is not None else 0.0,
        )
    AutoAction.AutoPilotAction(
        target=target,
        constraints=constraints,
        profile=profile,
        pid_gains=pid_gains,
        alliance_relative=alliance_relative,
    ).add()


def x_based_autopilot(
    target: Optional[AutoAction.APTarget] = None,
    *,
    target_pose: Optional[AutoAction.Pose2d] = None,
    entry_angle: Optional[AutoAction.Rotation2d] = None,
    velocity: Optional[float] = None,
    constraints: Optional[AutoAction.APConstraints] = None,
    profile: Optional[AutoAction.APProfile] = None,
    pid_gains: Optional[AutoAction.PIDGains] = None,
    alliance_relative: Optional[bool] = None,
) -> None:
    """Insert an XBasedAutoPilotAction into the current context."""
    if target is None:
        target = AutoAction.APTarget(
            reference=target_pose,
            entry_angle=entry_angle,
            velocity=velocity if velocity is not None else 0.0,
        )
    AutoAction.XBasedAutoPilotAction(
        target=target,
        constraints=constraints,
        profile=profile,
        pid_gains=pid_gains,
        alliance_relative=alliance_relative,
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
