"""
"Test Auto" autonomous routine.
"""

from __future__ import annotations

from auto_generator.src.routines import go_to_alliance_under_left_trench, go_to_center_under_left_trench_from_alliance


from .auto_action import Transform2d
from .auto_lib import auto
from .field_locations import FieldConstants
from .shorthands import (
    autopilot,
    deploy_intake,
    pose2d,
    pose3d_to_pose2d,
    reference,
    rotation2d,
    stow_intake,
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

    # go under left trench

    go_to_center_under_left_trench_from_alliance()

    # Deploy intake after safely out of trench to swipe center for balls

    deploy_intake()

    #swipe center for balls



    # Stow intake before moving to avoid potential collisions with field elements

    stow_intake()

    # go back under left trench to get back to alliance side

    go_to_alliance_under_left_trench()

    # Go to outpost

    autopilot(
        target_pose=FieldConstants.Outpost.center_point().to_pose2d().transform_by(
            Transform2d(
                translation=translation2d(x=1, y=0),
                rotation=rotation2d(degrees=180),
            ),
        ),
    )

    # Go climb on Left Tower Support

    reference("LeftClimbLineup")
