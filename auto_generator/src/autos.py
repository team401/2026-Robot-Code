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
    networkConfigurableWait,
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

def command(func: callable): # type: ignore

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
def from_bump_prepare_for_trench(angle=-90):
    autopilot(
        target_pose=pose2d(3.5, 7.55, angle),
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

@command
def _aggressive(use_depot=False, from_bump=False, shoot_preload=False, do_second_sweep=True):
    intake_cycle_time = 1 / 3
    intake_cycle_count = 1


    if shoot_preload:
        startShooting()
        wait(1.5)

    if from_bump:
        from_bump_prepare_for_trench()

    if shoot_preload:
        stopShooting()

    # Cycle 1

    # x_based_autopilot(
    #     target_pose=constants.left_trench_center_side_pose,
    #     velocity=constants.default_trench_velocity,
    #     entry_angle=rotation2d(0)
    # )
    with parallel():
        stow_intake()
        followPath(path_name="Left Trench To Center Intake In")

    with parallel():
        deploy_intake()
        followPath(path_name="Left Side Aggressive Sweep Intake In")

    with parallel():
        with sequence():
            wait(0.6)
            startShooting()
        followPath(path_name="Left Bump To Alliance")

    wait(0.1)

    autopilot(
        target_pose=pose2d(x=2.700, y=5.75,angle_degrees=-90)
    )

    wait(2.5)

    if do_second_sweep:
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

        wait(0.1)

        autopilot(
            target_pose=pose2d(x=2.700, y=5.75,angle_degrees=-90)
        )

        wait(1)

    if use_depot:
        autopilot(
            target_pose=pose2d(1.5, 5.9, -180),
            velocity=0.0,
            constraints=auto_action.APConstraints(
                velocity=2.0,
                acceleration=2.0,
                jerk=2.0
            ),
            pid_gains=PIDGains(
                k_p=1.5,
            )
        )

    cycle_intake(intake_cycle_time, intake_cycle_count)

@auto("Aggressive Depot")
def _aggressive_depot():
    _aggressive(True, False, False, True)

@auto("Aggressive No Depot")
def _aggressive_no_depot():
    _aggressive(False, False, False, True)

@auto("Aggressive Depot From Bump")
def _aggressive_depot():
    _aggressive(True, True, True, False)

@command
def _conservative(use_depot=False, from_bump=False, shoot_preload=False, do_second_sweep=True):
    intake_cycle_time = 0.5
    intake_cycle_count = 1

    if shoot_preload:
        startShooting()
        wait(1.5)

    if from_bump:
        from_bump_prepare_for_trench(angle=0)

    if shoot_preload:
        stopShooting()

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
        deploy_intake()
        followPath(path_name="Left Side Conservative Sweep")

    with parallel():
        with sequence():
            wait(0.6)
            startShooting()
        followPath(path_name="Left Bump To Alliance")

    wait(0.1)

    autopilot(
        target_pose=pose2d(x=2.700, y=5.75,angle_degrees=-90)
    )

    wait(2.5)

    # wait(1.0),
    # Cycle 2
    if do_second_sweep:
        cycle_intake(intake_cycle_time, intake_cycle_count)

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

        wait(0.1)

        autopilot(
            target_pose=pose2d(x=2.700, y=5.75,angle_degrees=-90)
        )

        wait(1)

    if use_depot:
        autopilot(
            target_pose=pose2d(1.5, 5.9, -180),
            velocity=0.0,
            constraints=auto_action.APConstraints(
                velocity=2.0,
                acceleration=2.0,
                jerk=2.0
            ),
            pid_gains=PIDGains(
                k_p=1.5,
            )
        )

    cycle_intake(intake_cycle_time, intake_cycle_count)


@auto("Conservative Depot")
def _conservative_depot():
    _conservative(True, False, False, True)

@auto("Conservative No Depot")
def _conservative_no_depot():
    _conservative(False, False, False, True)

@auto("Conservative Depot From Bump")
def _conservative_depot_from_bump():
    _conservative(True, True, True, False)


@auto("Follower")
def _follower():
    networkConfigurableWait("start", units.Second.of(1.0))
    _aggressive(True, False, False, True)
    networkConfigurableWait("middle", units.Second.of(2.0))
    _aggressive(True, False, False, True)

@auto("Center Depot")
def _center_depot():
    deploy_intake()
    startShooting()
    networkConfigurableWait("preload", units.Second.of(4.0))
    stopShooting()
    autopilot(
        target_pose=pose2d(0.802, 5.254, 135),
        constraints=auto_action.APConstraints(
            velocity=2.0,
            acceleration=2.0,
            jerk=2.0
        ),
        pid_gains=PIDGains(
            k_p=1.5,
        )
    )
    followPath("Intake Depot")
    autopilot(
        target_pose=pose2d(1.3, 7.25, 135),
        constraints=auto_action.APConstraints(
            velocity=2.0,
            acceleration=2.0,
            jerk=2.0
        ),
        pid_gains=PIDGains(
            k_p=1.5,
        )
    )
    startShooting()
