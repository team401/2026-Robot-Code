"""
Python equivalent of newTestAuto.ts.

Defines the "Test Auto" autonomous routine.
"""

from __future__ import annotations

from . import auto_lib as AutoLib
from .field_locations import FieldConstants
from .shorthands import (
    autopilot,
    pose2d,
    pose3d_to_pose2d,
    reference,
    rotation2d,
    translate2d_pose,
    translation2d,
    wait,
    x_based_autopilot_action,
)

# TODO: Add alliance-relative coordinate utilities.
# TODO: Replace placeholder coordinates with real field positions.
# TODO: Switch to AutoPilotAction with entry angle and exit velocity for trench segments.


def _build_test_auto() -> None:
    # NOTE: These coordinates are placeholders and should be replaced with actual field positions.
    autopilot(target_pose=FieldConstants.Alliance.center)  # Center of Alliance Zone
    x_based_autopilot_action(
        target_pose=pose3d_to_pose2d(FieldConstants.RightTrench.opening_floor_center()),
        entry_angle=rotation2d(angle_degrees=180),
        velocity=100.0,
    )  # Middle of trench (right side)
    autopilot(
        target_pose=pose2d(
            x=FieldConstants.LinesVertical.center,
            y=FieldConstants.LinesHorizontal.center,
        ),
        entry_angle=rotation2d(angle_degrees=-90),
    )  # Center of field
    wait(seconds=1.0)  # Wait for 1 second
    x_based_autopilot_action(
        target_pose=pose3d_to_pose2d(FieldConstants.RightTrench.opening_floor_center()),
        entry_angle=rotation2d(angle_degrees=0),
        velocity=100.0,
    )
    autopilot(
        target_pose=translate2d_pose(
            p=FieldConstants.Alliance.center,
            t=translation2d(x=0, y=0),
        ),
        entry_angle=rotation2d(angle_degrees=-90),
    )
    reference("LeftClimbLineup")


AutoLib.auto("Test Auto", _build_test_auto)
