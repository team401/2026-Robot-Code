"""
Python equivalent of Constants.ts.

Robot-specific constants used by the auto routines.
"""

from __future__ import annotations

from . import auto_action as AutoAction
from .field_locations import FieldConstants
from .shorthands import (
    rotation2d,
    transform2d_pose,
    translation2d,
    translation2d_to_pose2d,
)

# TODO: Maybe make these loaded from the constants files in the main robot code
# instead of hardcoded here, to avoid duplication and potential inconsistencies.

climb_offset = AutoAction.Transform2d(
    translation=translation2d(x=0.41, y=0.0225),
    rotation=rotation2d(angle_degrees=-90.0),
)

climb_constraints = AutoAction.APConstraints(
    velocity=2.0,
    acceleration=2.0,
    jerk=0.1,
)

left_climb_location = transform2d_pose(
    p=translation2d_to_pose2d(FieldConstants.Tower.left_upright()),
    transform=climb_offset,
)

right_climb_location = transform2d_pose(
    p=translation2d_to_pose2d(FieldConstants.Tower.right_upright()),
    transform=climb_offset,
)

climb_lineup_velocity = 100.0

climb_left_lineup_entry_angle = AutoAction.Rotation2d(degrees=0)
climb_right_lineup_entry_angle = climb_left_lineup_entry_angle
