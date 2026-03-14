"""
"Test Auto" autonomous routine.
"""

from __future__ import annotations

from .routines import go_to_alliance_under_left_trench, go_to_alliance_under_right_trench, go_to_center_under_left_trench_from_alliance, go_to_center_under_right_trench_from_alliance

from .auto_action import APConstraints, Transform2d
from .auto_lib import auto, parallel, routines, sequence
from .field_locations import FieldConstants
from .shorthands import (
    autopilot,
    deploy_intake,
    pose2d,
    rotation2d,
    transform2d,
    startShooting,
    stopShooting,
    stow_intake,
    translation2d,
    wait
)
from . import units
from . import constants
from . import auto_action

# TODO: Add alliance-relative coordinate utilities.
# TODO: Replace placeholder coordinates with real field positions.
# TODO: Switch to AutoPilotAction with entry angle and exit velocity for trench segments.


@auto("Right Side Auto")
def _right_side_auto():

    startShooting()

    wait(3)
    # go under right trench

    # Disable first entry angle so that if we start a bit to off we don't have to worry about it
    # Trying to make a loop
    # And disabling the second entry angle because if we start past the first trench pose we go back to the first pose
    # and then we try to loop to because our entry angle would be wrong.
    stopShooting()
    go_to_center_under_right_trench_from_alliance(first_entry_angle=None, second_entry_angle=None)


    # Go to a pose that is clear of the trench and has a good angle to intake balls from the center, and then we can drive to the center from there to intake the balls.

    autopilot(
        target_pose=constants.right_trench_center_side_pose.transform_by(
            transform2d(
                translation=translation2d(x=0.7, y=0),
            )
        ),
        velocity=constants.default_trench_velocity,
    )

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

    # Move to a clean pose to go to center to shoot from.

    autopilot(
        target_pose=constants.right_trench_alliance_side_pose.transform_by(
            transform2d(
                translation=translation2d(x=-0.7, y=0),
            )
        ),
        velocity=constants.default_trench_velocity,
    )

    autopilot(
        target_pose=FieldConstants.Alliance.center,
        entry_angle=rotation2d(degrees=-90),
        profile=auto_action.APProfile(
            constraints=None,
            error_theta=units.Degree.of(360),
            error_x_y=units.Meter.of(0.15),
            beeline_radius=units.Meter.of(0.5)
        )
    )

    # Maybe consider when we want to start shooting.
    # Maybe we want a separate action to warm up the shooter and then start shooting when we are in
    startShooting()

    wait(3)

    stow_intake()

    wait(2)

    # Avoid wasting time stopping climb when we could be starting to climb so we
    # can just do both at the same time.
    with parallel():
        stopShooting()
        routines.RightClimb()


@auto("Left Side Auto")
def _left_side_auto():

    startShooting()

    wait(3)

    stopShooting()
    # go under left trench

    # Disable first entry angle so that if we start a bit to off we don't have to worry about it
    # Trying to make a loop
    # And disabling the second entry angle because if we start past the first trench pose we go back to the first pose
    # and then we try to loop to because our entry angle would be wrong.
    go_to_center_under_left_trench_from_alliance(first_entry_angle=None, second_entry_angle=None)


    # Go to a pose that is clear of the trench and has a good angle to intake balls from the center, and then we can drive to the center from there to intake the balls.

    autopilot(
        target_pose=constants.left_trench_center_side_pose.transform_by(
            transform2d(
                translation=translation2d(x=0.7, y=0),
            )
        ),
        velocity=constants.default_trench_velocity,
    )

    # Deploy intake after safely out of trench to swipe center for balls

    deploy_intake()

    #swipe center for balls

    autopilot(
        target_pose=FieldConstants.Center.center_point().to_pose2d().transform_by(
            transform2d(
                translation=translation2d(x=-0.2, y=-1),
                rotation=rotation2d(degrees=-90),
            )
        ),
        entry_angle=rotation2d(degrees=90),
        profile=auto_action.APProfile(
            constraints=APConstraints(
                velocity=3,
                acceleration=2,
                jerk=2
            ),
            error_theta=units.Degree.of(360),
            error_x_y=units.Meter.of(0.15),
            beeline_radius=units.Meter.of(0.5)
        )
    )

    # go back under left trench to get back to alliance side

    go_to_alliance_under_left_trench(rotation=rotation2d(degrees=0))

    # Move to a clean pose to go to center to shoot from.

    with parallel():

        startShooting()

        with sequence():

            autopilot(
                target_pose=constants.left_trench_alliance_side_pose.transform_by(
                    transform2d(
                        translation=translation2d(x=-0.7, y=0),
                    )
                ),
                velocity=constants.default_trench_velocity,
            )

            # Replace this with the depot
            
            # autopilot(
            #     target_pose=pose2d(x=2, y=6.8, angle_degrees=180),
            #     entry_angle=rotation2d(90),
            #     velocity=constants.default_trench_velocity
            # )

            autopilot(
                target_pose=FieldConstants.Depot.depot_center().to_pose2d().transform_by(
                    transform2d(
                        translation=translation2d(x=0.75),
                        rotation=rotation2d(180)
                    )
                ),
                entry_angle=rotation2d(degrees=0),
                rotation_radius=units.Meter.of(2)
            )

    wait(3)

    stow_intake()

    wait(2)

    autopilot(
        target_pose=FieldConstants.Alliance.center,
        entry_angle=rotation2d(degrees=90),
    )

    # Avoid wasting time to shooting when we could be starting to climb so we
    # can just do both at the same time.
    with parallel():
        stopShooting()
        routines.LeftClimb()