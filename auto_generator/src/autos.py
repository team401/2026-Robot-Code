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
    wait,
    x_based_autopilot
)
from . import units
from . import constants
from . import auto_action

# TODO: Add alliance-relative coordinate utilities.
# TODO: Replace placeholder coordinates with real field positions.
# TODO: Switch to AutoPilotAction with entry angle and exit velocity for trench segments.


@auto("Literally just shoot Auto", can_be_mirrored=False, should_be_flipped=False)
def _literally_just_shoot():
    startShooting()


@auto("Testing Path Planner", can_be_mirrored=False)
def _testing_path_planner():
    auto_action.FollowPathPlannerPath(path_name="New Path").add()

def cycle_intake(time, count):
    delay_each = time/2/count
    for i in range(count):
        stow_intake()
        wait(delay_each)
        deploy_intake()
        wait(delay_each)

@auto("Double Swipe (opp has auto)", can_be_mirrored=True)
def _test_path_planner_left_auto():
    intake_cycle_time = 2
    intake_cycle_count = 2

    # Cycle 1

    x_based_autopilot(
        target_pose=constants.left_trench_center_side_pose,
        velocity=constants.default_trench_velocity,
        entry_angle=rotation2d(0)
    )

    with parallel():
        deploy_intake()
        auto_action.FollowPathPlannerPath(path_name="Left Side Close Sweep").add()

    with parallel():
        with sequence():
            wait(0.6)
            startShooting()
        auto_action.FollowPathPlannerPath(path_name="Left Bump To Alliance").add()

    with parallel():

        auto_action.FollowPathPlannerPath(path_name="Turn 180").add()

        with sequence():
            startShooting()

            wait(1.0)

            cycle_intake(intake_cycle_time, intake_cycle_count)

    #wait(1.0)

    # Cycle 2

    autopilot(
        target_pose=constants.left_trench_alliance_side_pose.plus(
            transform2d(translation=translation2d(x=-0.5, y=0))
        ),
        velocity=constants.default_trench_velocity,
        entry_angle=rotation2d(0),
        constraints=auto_action.APConstraints(
            velocity=2.0,
            acceleration=2.0,
            jerk=2.0
        )
    )

    with parallel():

        stopShooting()

        go_to_center_under_left_trench_from_alliance()

    with parallel():
        deploy_intake()
        auto_action.FollowPathPlannerPath(path_name="Left Side Close 2nd Sweep").add()

    with parallel():
        with sequence():
            wait(0.4)
            startShooting()
        auto_action.FollowPathPlannerPath(path_name="Left Bump To Alliance").add()

    wait(1)

    cycle_intake(intake_cycle_time, intake_cycle_count)

    stow_intake()


