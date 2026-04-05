"""
"Test Auto" autonomous routine.
"""

from __future__ import annotations

from .routines import go_to_alliance_under_left_trench, go_to_alliance_under_right_trench, go_to_center_under_left_trench_from_alliance, go_to_center_under_left_trench_from_alliance_intake_in, go_to_center_under_right_trench_from_alliance

from .auto_action import APConstraints, PIDGains, Transform2d
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
    x_based_autopilot,
    followPath
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

def command(func: callable):

    def _inner(*args, **kwargs):
        with sequence():
            func(*args, **kwargs)

    return _inner

@command
def cycle_intake(time, count):
    delay_each = time/2/count
    for i in range(count):
        stow_intake()
        wait(delay_each)
        deploy_intake()
        wait(delay_each)

@command
def first_intake_deploy():
    deploy_intake()
    wait(0.2)
    stow_intake()
    wait(0.6)
    deploy_intake()

@auto("Double Swipe (opp has auto)")
def _double_swipe_opp_has_auto():
    intake_cycle_time = 2
    intake_cycle_count = 5

    # Cycle 1

    # x_based_autopilot(
    #     target_pose=constants.left_trench_center_side_pose,
    #     velocity=constants.default_trench_velocity,
    #     entry_angle=rotation2d(0)
    # )
    with parallel():
        stow_intake()
        followPath(path_name="Left Trench To Center")

    with parallel():
        first_intake_deploy()
        followPath(path_name="Left Side Close Sweep")

    with parallel():
        with sequence():
            wait(0.6)
            startShooting()
        followPath(path_name="Left Bump To Alliance")

    with parallel():

        # followPath(path_name="Turn 180")

        with sequence():
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
        ),
        pid_gains=PIDGains(
            k_p=1.5,
        )
    )

    with parallel():

        stopShooting()

        go_to_center_under_left_trench_from_alliance()

    with parallel():
        deploy_intake()
        followPath(path_name="Left Side Close 2nd Sweep")

    with parallel():
        with sequence():
            wait(0.4)
            startShooting()
        followPath(path_name="Left Bump To Alliance")

    wait(1)

    cycle_intake(intake_cycle_time, intake_cycle_count)

@auto("Double Swipe intake in")
def _double_swipe_intake_in():
    intake_cycle_time = 2
    intake_cycle_count = 5

    # Cycle 1

    # x_based_autopilot(
    #     target_pose=constants.left_trench_center_side_pose,
    #     velocity=constants.default_trench_velocity,
    #     entry_angle=rotation2d(0)
    # )
    with parallel():
        stow_intake()
        followPath(path_name="Left Trench To Center")

    with parallel():
        first_intake_deploy()
        followPath(path_name="Left Side Close Sweep")

    with parallel():
        with sequence():
            wait(0.6)
            startShooting()
        followPath(path_name="Left Bump To Alliance")

    # wait(0.1)

    # autopilot(
    #     target_pose=pose2d(x=2.700, y=5.75,angle_degrees=-90)
    # )

    with parallel():

        # followPath(path_name="Turn 180")

        with sequence():
            wait(1.0)

            cycle_intake(intake_cycle_time, intake_cycle_count)

            cycle_intake(intake_cycle_time, intake_cycle_count)

    # wait(1.0),
    # Cycle 2

    autopilot(
        target_pose=pose2d(3.5, 7.55, -90),
        velocity=constants.default_trench_velocity,
        entry_angle=rotation2d(0),
        constraints=auto_action.APConstraints(
            velocity=2.0,
            acceleration=2.0,
            jerk=2.0
        ),
        pid_gains=PIDGains(
            k_p=1.5,
        )
    )

    with parallel():

        stopShooting()

        go_to_center_under_left_trench_from_alliance_intake_in()

    with parallel():
        deploy_intake()
        followPath(path_name="Left Side Close 2nd Sweep Intake In")

    with parallel():
        with sequence():
            wait(0.4)
            startShooting()
        followPath(path_name="Left Bump To Alliance")

    wait(1)

    cycle_intake(intake_cycle_time, intake_cycle_count)

    cycle_intake(intake_cycle_time, intake_cycle_count)

    cycle_intake(intake_cycle_time, intake_cycle_count)
    
