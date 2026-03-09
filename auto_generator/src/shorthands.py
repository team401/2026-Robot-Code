"""
Python equivalent of Shorthands.ts.

Provides geometry helper functions and command shorthand wrappers for the
auto builder DSL.
"""

from __future__ import annotations

from typing import Callable, Optional

from . import auto_action as AutoAction
from . import auto_lib as AutoLib
from . import units as Units


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
    return AutoAction.Pose2d(
        translation=t,
        rotation=rotation2d(),
    )


def pose2d_to_pose3d(
    p: AutoAction.Pose2d, z: float = 0.0
) -> AutoAction.Pose3d:
    return AutoAction.Pose3d(
        translation=translation3d(
            x=p.translation.x if p.translation else 0.0,
            y=p.translation.y if p.translation else 0.0,
            z=z,
        ),
        rotation=rotation3d(
            yaw_degrees=p.rotation.degrees if p.rotation else 0.0,
        ),
    )


def pose3d_to_pose2d(p: AutoAction.Pose3d) -> AutoAction.Pose2d:
    return AutoAction.Pose2d(
        translation=translation2d(
            x=p.translation.x if p.translation else 0.0,
            y=p.translation.y if p.translation else 0.0,
        ),
        rotation=rotation2d(
            angle_degrees=p.rotation.yaw if p.rotation else 0.0,
        ),
    )


def translate2d_pose(
    p: AutoAction.Pose2d, t: AutoAction.Translation2d
) -> AutoAction.Pose2d:
    current_x = p.translation.x if p.translation else 0.0
    current_y = p.translation.y if p.translation else 0.0
    dx = t.x if t else 0.0
    dy = t.y if t else 0.0
    return AutoAction.Pose2d(
        translation=translation2d(x=current_x + dx, y=current_y + dy),
        rotation=p.rotation if p.rotation else rotation2d(),
    )


def transform2d_pose(
    p: AutoAction.Pose2d, transform: AutoAction.Transform2d
) -> AutoAction.Pose2d:
    current_x = p.translation.x if p.translation else 0.0
    current_y = p.translation.y if p.translation else 0.0
    dx = transform.translation.x if transform.translation else 0.0
    dy = transform.translation.y if transform.translation else 0.0
    current_theta = p.rotation.degrees if p.rotation else 0.0
    d_theta = transform.rotation.degrees if transform.rotation else 0.0
    return AutoAction.Pose2d(
        translation=translation2d(x=current_x + dx, y=current_y + dy),
        rotation=rotation2d(angle_degrees=current_theta + d_theta),
    )


# ---------------------------------------------------------------------------
# Primitive commands
# ---------------------------------------------------------------------------


def wait(seconds: float) -> AutoAction.Wait:
    return AutoAction.Wait(delay=Units.Second.of(seconds)).add()


def print_msg(message: str) -> AutoAction.Print:
    return AutoAction.Print(message=message).add()


def autopilot(
    target_pose: AutoAction.Pose2d,
    entry_angle: Optional[AutoAction.Rotation2d] = None,
    velocity: Optional[float] = None,
) -> AutoAction.AutoPilotAction:
    return AutoAction.AutoPilotAction(
        target=AutoAction.APTarget(
            reference=target_pose,
            entry_angle=entry_angle,
            velocity=velocity if velocity is not None else 0.0,
        ),
    ).add()


def x_based_autopilot_action(
    target_pose: AutoAction.Pose2d,
    entry_angle: Optional[AutoAction.Rotation2d] = None,
    velocity: Optional[float] = None,
) -> AutoAction.XBasedAutoPilotAction:
    return AutoAction.XBasedAutoPilotAction(
        target=AutoAction.APTarget(
            reference=target_pose,
            entry_angle=entry_angle,
            velocity=velocity if velocity is not None else 0.0,
        ),
    ).add()


# ---------------------------------------------------------------------------
# Container helpers
# ---------------------------------------------------------------------------


def sequence(build: Callable[[], None]) -> None:
    AutoLib.with_container(AutoAction.Sequence(), build)


def parallel(build: Callable[[], None]) -> None:
    AutoLib.with_container(AutoAction.Parallel(), build)


def race(build: Callable[[], None]) -> None:
    AutoLib.with_container(AutoAction.Race(), build)


def reference(auto_name: str) -> AutoAction.AutoReference:
    return AutoAction.AutoReference(name=auto_name).add()
