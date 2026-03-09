"""
"Test Auto" autonomous routine.
"""

from __future__ import annotations


from .auto_action import Transform2d
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
    autopilot(target_pose=FieldConstants.Alliance.center)
    x_based_autopilot(
        target_pose=FieldConstants.RightTrench.opening_floor_center().to_pose2d(),
        entry_angle=rotation2d(angle_degrees=180),
        velocity=100.0,
    )
    autopilot(
        target_pose=pose2d(
            x=FieldConstants.LinesVertical.center,
            y=FieldConstants.LinesHorizontal.center,
            angle_degrees=90,
        ),
        entry_angle=rotation2d(angle_degrees=-90),
        rotation_radius=Meter.of(2.5),
    )
    x_based_autopilot(
        target_pose=FieldConstants.RightTrench.opening_floor_center().to_pose2d().transform_by(
            Transform2d(
                translation=translation2d(x=0, y=0),
                rotation=rotation2d(angle_degrees=180),
            )
        ),
        entry_angle=rotation2d(angle_degrees=0),
        velocity=100.0,
    )
    autopilot(
        target_pose=FieldConstants.Outpost.center_point().to_pose2d().transform_by(
            Transform2d(
                translation=translation2d(x=1, y=0),
                rotation=rotation2d(angle_degrees=180),
            ),
        ),
    )
    reference("LeftClimbLineup")
