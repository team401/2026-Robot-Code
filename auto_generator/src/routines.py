"""
Reusable auto subroutines (e.g. climb lineup).
"""

from __future__ import annotations

from typing import Optional

from . import auto_action as AutoAction
from .auto_lib import auto, parallel, routine, routines, sequence
from . import constants as Constants
from .field_locations import FieldConstants
from .shorthands import (
    autopilot,
    climb_hang,
    climb_search,
    rotation2d,
    wait,
    x_based_autopilot,
)

# I really want to rename these commands or do something about them

def go_to_alliance_under_left_trench(velocity = Constants.default_trench_velocity, rotation = rotation2d(0), first_entry_angle = rotation2d(0), second_entry_angle = rotation2d(0)):
    transform = AutoAction.Transform2d(
        rotation=rotation,
    )
    with sequence():
        x_based_autopilot(
            target_pose=Constants.left_trench_center_side_pose.transform_by(transform),
            velocity=velocity,
            entry_angle=first_entry_angle
        )
        x_based_autopilot(
            target_pose=Constants.left_trench_alliance_side_pose.transform_by(transform),
            velocity=velocity,
            entry_angle=second_entry_angle
        )

def go_to_alliance_under_right_trench(velocity = Constants.default_trench_velocity, rotation = rotation2d(0), first_entry_angle = rotation2d(0), second_entry_angle = rotation2d(0)):
    transform = AutoAction.Transform2d(
        rotation=rotation,
    )
    with sequence():
        x_based_autopilot(
            target_pose=Constants.right_trench_center_side_pose.transform_by(transform),
            velocity=velocity,
            entry_angle=first_entry_angle
        )
        x_based_autopilot(
            target_pose=Constants.right_trench_alliance_side_pose.transform_by(transform),
            velocity=velocity,
            entry_angle=second_entry_angle
        )

def go_to_center_under_left_trench_from_alliance(velocity = Constants.default_trench_velocity, rotation = rotation2d(), first_entry_angle = rotation2d(180), second_entry_angle = rotation2d(180)):
    transform = AutoAction.Transform2d(
        rotation=rotation,
    )
    with sequence():
        x_based_autopilot(
            target_pose=Constants.left_trench_alliance_side_pose.transform_by(transform),
            velocity=velocity,
            entry_angle=first_entry_angle
        )
        x_based_autopilot(
            target_pose=Constants.left_trench_center_side_pose.transform_by(transform),
            velocity=velocity,
            entry_angle=second_entry_angle
        )

def go_to_center_under_right_trench_from_alliance(velocity = Constants.default_trench_velocity, rotation = rotation2d(0), first_entry_angle = rotation2d(180), second_entry_angle = rotation2d(180)):
    transform = AutoAction.Transform2d(
        rotation=rotation,
    )
    with sequence():
        x_based_autopilot(
            target_pose=Constants.right_trench_alliance_side_pose.transform_by(transform),
            velocity=velocity,
            entry_angle=first_entry_angle
        )
        x_based_autopilot(
            target_pose=Constants.right_trench_center_side_pose.transform_by(transform),
            velocity=velocity,
            entry_angle=second_entry_angle
        )


def _climb(
    target_pose: AutoAction.Pose2d,
    entry_angle: Optional[AutoAction.Rotation2d] = None,
    velocity: Optional[float] = None,
) -> None:
    """Reusable climb lineup subroutine. Call inside a context."""
    with parallel():
        with sequence():
            # x_based_autopilot(
            #     target_pose=FieldConstants.Alliance.center.transform_by(Constants.climb_offset),
            #     constraints=Constants.climb_constraints
            # )
            x_based_autopilot(
                target_pose=target_pose,
                entry_angle=entry_angle,
                constraints=Constants.climb_constraints
            )
        climb_search()
    wait(0.5)
    climb_hang()

# Register climb lineup routines


@routine("LeftClimb")
def _left_climb():
    _climb(
        target_pose=Constants.left_climb_location,
        entry_angle=Constants.climb_left_lineup_entry_angle
    )


@routine("RightClimb")
def _right_climb():
    _climb(
        target_pose=Constants.right_climb_location,
        entry_angle=Constants.climb_right_lineup_entry_angle
    )
