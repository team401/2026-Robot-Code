"""
"Test Auto" autonomous routine.
"""

from __future__ import annotations

from .routines import go_to_alliance_under_right_trench, go_to_center_under_right_trench_from_alliance

from .auto_action import APConstraints, Transform2d
from .auto_lib import auto, parallel, routines
from .field_locations import FieldConstants
from .shorthands import (
    autopilot,
    deploy_intake,
    rotation2d,
    transform2d,
    startShooting,
    stopShooting,
    stow_intake,
    translation2d,
    wait
)
from . import constants

# TODO: Add alliance-relative coordinate utilities.
# TODO: Replace placeholder coordinates with real field positions.
# TODO: Switch to AutoPilotAction with entry angle and exit velocity for trench segments.


@auto("Test Auto")
def _test_auto():
    # autopilot(target_pose=FieldConstants.Alliance.center)

    # go under right trench

    go_to_center_under_right_trench_from_alliance()


    # Deploy intake after safely out of trench to swipe center for balls

    deploy_intake()

    #swipe center for balls

    autopilot(
        target_pose=FieldConstants.Center.center_point().to_pose2d().transform_by(
            transform2d(
                translation=translation2d(x=-0.2, y=0),
                rotation=rotation2d(degrees=90),
            )
        ),
        entry_angle=rotation2d(degrees=-90),
        constraints=APConstraints(
            velocity=3,
            acceleration=2,
            jerk=2
        )
    )

    # go back under right trench to get back to alliance side

    go_to_alliance_under_right_trench(rotation=rotation2d(degrees=0))

    # Go to outpost

    # autopilot(
    #     target_pose=FieldConstants.Outpost.center_point().to_pose2d().transform_by(
    #         Transform2d(
    #             translation=translation2d(x=1, y=0),
    #             rotation=rotation2d(degrees=0),
    #         ),
    #     ),
    # )

    with parallel():
        autopilot(target_pose=FieldConstants.Alliance.center)

        startShooting()

    wait(3)

    stow_intake()

    wait(2)

    stopShooting()

    # Go climb on Left Tower Support

    routines.LeftClimb()
