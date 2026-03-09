"""
Python equivalent of Routines.ts.

Defines reusable auto subroutines (e.g. climb lineup).
"""

from __future__ import annotations

from typing import Optional

from . import auto_action as AutoAction
from . import auto_lib as AutoLib
from . import constants as Constants
from .field_locations import FieldConstants
from .shorthands import (
    autopilot,
    parallel,
    sequence,
    transform2d_pose,
    x_based_autopilot_action,
)


def climb_lineup(
    target_pose: AutoAction.Pose2d,
    entry_angle: Optional[AutoAction.Rotation2d] = None,
    velocity: Optional[float] = None,
) -> None:
    def _inner():
        def _drive_and_search():
            def _drive_sequence():
                x_based_autopilot_action(
                    target_pose=transform2d_pose(
                        p=FieldConstants.Alliance.center,
                        transform=Constants.climb_offset,
                    )
                )
                autopilot(
                    target_pose=target_pose,
                    entry_angle=entry_angle,
                    velocity=velocity,
                )

            sequence(_drive_sequence)
            AutoAction.ClimbSearchAction().add()

        parallel(_drive_and_search)
        AutoAction.ClimbHangAction().add()

    sequence(_inner)


def _make_climb_lineup(
    target_pose: AutoAction.Pose2d,
    entry_angle: Optional[AutoAction.Rotation2d] = None,
    velocity: Optional[float] = None,
):
    """Returns a callable suitable for AutoLib.auto()."""
    def _builder():
        climb_lineup(
            target_pose=target_pose,
            entry_angle=entry_angle,
            velocity=velocity,
        )
    return _builder


# Register climb lineup autos
AutoLib.auto(
    "LeftClimbLineup",
    _make_climb_lineup(
        target_pose=Constants.left_climb_location,
        entry_angle=Constants.climb_left_lineup_entry_angle,
        velocity=Constants.climb_lineup_velocity,
    ),
)

AutoLib.auto(
    "RightClimbLineup",
    _make_climb_lineup(
        target_pose=Constants.right_climb_location,
        entry_angle=Constants.climb_right_lineup_entry_angle,
        velocity=Constants.climb_lineup_velocity,
    ),
)
