"""
"Test Auto" autonomous routine.
"""

from __future__ import annotations

from .routines import go_to_alliance_under_right_trench, go_to_center_under_right_trench_from_alliance

from .auto_action import Transform2d
from .auto_lib import auto, routines
from .field_locations import FieldConstants
from .shorthands import (
    autopilot,
    deploy_intake,
    rotation2d,
    transform2d,
    startShooting,
    stopShooting,
    stow_intake,
    translation2d
)

# TODO: Add alliance-relative coordinate utilities.
# TODO: Replace placeholder coordinates with real field positions.
# TODO: Switch to AutoPilotAction with entry angle and exit velocity for trench segments.


@auto("Test Auto")
def _test_auto():
    autopilot(target_pose=FieldConstants.Alliance.center)

    # go under right trench

    go_to_center_under_right_trench_from_alliance()

    # Deploy intake after safely out of trench to swipe center for balls

    deploy_intake()

    #swipe center for balls

    autopilot(
        target_pose=FieldConstants.Center.center_point().to_pose2d().transform_by(
            transform2d(
                translation=translation2d(x=-1, y=1),
                rotation=rotation2d(degrees=90),
            )
        ),
        entry_angle=rotation2d(degrees=90),
    )

    # Stow intake before moving to avoid potential collisions with field elements

    stow_intake()

    # go back under right trench to get back to alliance side

    go_to_alliance_under_right_trench()

    # Go to outpost

    startShooting()

    autopilot(
        target_pose=FieldConstants.Outpost.center_point().to_pose2d().transform_by(
            Transform2d(
                translation=translation2d(x=1, y=0),
                rotation=rotation2d(degrees=180),
            ),
        ),
    )

    stopShooting()

    # Go climb on Left Tower Support

    routines.LeftClimb()
