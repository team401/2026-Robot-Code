"""
Reusable auto subroutines (e.g. climb lineup).
"""

from __future__ import annotations

from typing import Optional

from . import auto_action as AutoAction
from .auto_lib import auto, parallel, sequence
from . import constants as Constants
from .field_locations import FieldConstants
from .shorthands import (
    autopilot,
    climb_hang,
    climb_search,
    transform2d_pose,
    x_based_autopilot,
)


def climb_lineup(
    target_pose: AutoAction.Pose2d,
    entry_angle: Optional[AutoAction.Rotation2d] = None,
    velocity: Optional[float] = None,
) -> None:
    """Reusable climb lineup subroutine. Call inside a context."""
    with sequence():
        with parallel():
            with sequence():
                x_based_autopilot(
                    target_pose=transform2d_pose(
                        p=FieldConstants.Alliance.center,
                        transform=Constants.climb_offset,
                    ),
                )
                autopilot(
                    target_pose=target_pose,
                    entry_angle=entry_angle,
                    velocity=velocity,
                )
            climb_search()
        climb_hang()


# Register climb lineup autos


@auto("LeftClimbLineup")
def _left_climb_lineup():
    climb_lineup(
        target_pose=Constants.left_climb_location,
        entry_angle=Constants.climb_left_lineup_entry_angle,
        velocity=Constants.climb_lineup_velocity,
    )


@auto("RightClimbLineup")
def _right_climb_lineup():
    climb_lineup(
        target_pose=Constants.right_climb_location,
        entry_angle=Constants.climb_right_lineup_entry_angle,
        velocity=Constants.climb_lineup_velocity,
    )
