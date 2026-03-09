"""
"Test Auto" autonomous routine.
"""

from __future__ import annotations

from . import auto_action as AutoAction
from .auto_lib import auto
from .field_locations import FieldConstants
from .shorthands import (
    autopilot,
    pose2d,
    pose3d_to_pose2d,
    reference,
    rotation2d,
    transform2d_pose,
    translation2d,
    translation2d_to_pose2d,
    x_based_autopilot,
)
from .units import Meter

# TODO: Add alliance-relative coordinate utilities.
# TODO: Replace placeholder coordinates with real field positions.
# TODO: Switch to AutoPilotAction with entry angle and exit velocity for trench segments.


@auto("Test Auto")
def _test_auto():
    # NOTE: These coordinates are placeholders and should be replaced with actual field positions.
    autopilot(target_pose=FieldConstants.Alliance.center)  # Center of Alliance Zone
    x_based_autopilot(
        target_pose=pose3d_to_pose2d(FieldConstants.RightTrench.opening_floor_center()),
        entry_angle=rotation2d(angle_degrees=180),
        velocity=100.0,
    )  # Middle of trench (right side)
    autopilot(
        target_pose=pose2d(
            x=FieldConstants.LinesVertical.center,
            y=FieldConstants.LinesHorizontal.center,
            angle_degrees=90,
        ),
        entry_angle=rotation2d(angle_degrees=-90),
        rotation_radius=Meter.of(2.5),
    )  # Center of field
    x_based_autopilot(
        target_pose=transform2d_pose(
            p=pose3d_to_pose2d(FieldConstants.RightTrench.opening_floor_center()),
            transform=AutoAction.Transform2d(
                translation=translation2d(x=0, y=0),
                rotation=rotation2d(angle_degrees=180),
            ),
        ),
        entry_angle=rotation2d(angle_degrees=0),
        velocity=100.0,
    )
    autopilot(
        target_pose=transform2d_pose(
            p=translation2d_to_pose2d(FieldConstants.Outpost.center_point()),
            transform=AutoAction.Transform2d(
                translation=translation2d(x=1, y=0),
                rotation=rotation2d(angle_degrees=180),
            ),
        ),
    )
    reference("LeftClimbLineup")
