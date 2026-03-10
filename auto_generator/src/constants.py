"""
Python equivalent of Constants.ts.

Robot-specific constants used by the auto routines.
"""

from __future__ import annotations

from . import auto_action as AutoAction
from .auto_action import (
    APConstraints,
    Transform2d
)
from .field_locations import FieldConstants
from .shorthands import (
    rotation2d,
    translation2d,
)

# TODO: Maybe make these loaded from the constants files in the main robot code
# instead of hardcoded here, to avoid duplication and potential inconsistencies.

climb_offset = Transform2d(
    translation=translation2d(x=0.41, y=0.0225),
    rotation=rotation2d(degrees=-90.0),
)

climb_constraints = APConstraints(
    velocity=2.0,
    acceleration=2.0,
    jerk=0.1,
)

left_climb_location = FieldConstants.Tower.left_upright().to_pose2d().transform_by(climb_offset)

right_climb_location = FieldConstants.Tower.right_upright().to_pose2d().transform_by(climb_offset)

climb_lineup_velocity = 100.0

climb_left_lineup_entry_angle = rotation2d(degrees=0)
climb_right_lineup_entry_angle = climb_left_lineup_entry_angle


default_trench_velocity = 3.0

center_side_trench_offset = translation2d(x=1.0, y=0.0)
alliance_side_trench_offset = translation2d(x=-1.0, y=0.0)

left_trench_center_side_pose = FieldConstants.LeftTrench.opening_floor_center().to_pose2d().translate_by(center_side_trench_offset)
right_trench_center_side_pose = FieldConstants.RightTrench.opening_floor_center().to_pose2d().translate_by(center_side_trench_offset)

left_trench_alliance_side_pose = FieldConstants.LeftTrench.opening_floor_center().to_pose2d().translate_by(alliance_side_trench_offset)
right_trench_alliance_side_pose = FieldConstants.RightTrench.opening_floor_center().to_pose2d().translate_by(alliance_side_trench_offset)
