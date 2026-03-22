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


@auto("Literally just shoot Auto", can_be_mirrored=False, should_be_flipped=False)
def _literally_just_shoot():
    startShooting()


@auto("Testing Path Planner", can_be_mirrored=False)
def _testing_path_planner():
    auto_action.FollowPathPlannerPath(path_name="New Path").add()

@auto("Test Path Planner Left Auto", can_be_mirrored=False)
def _test_path_planner_left_auto():

    auto_action.FollowPathPlannerPath(path_name="Left Trench To Center").add()

    deploy_intake()

    auto_action.FollowPathPlannerPath(path_name="Left Side Sweep").add()
    auto_action.FollowPathPlannerPath(path_name="Left Trench To Alliance").add()
    auto_action.FollowPathPlannerPath(path_name="Left Trench To Alliance Center").add()

    startShooting()

    wait(1.5)

    stow_intake()

    stopShooting()

    deploy_intake()

    auto_action.FollowPathPlannerPath(path_name="Alliance Center To Left Trench").add()
    auto_action.FollowPathPlannerPath(path_name="Left Trench To Center").add()
    auto_action.FollowPathPlannerPath(path_name="Left Side Sweep").add()
    auto_action.FollowPathPlannerPath(path_name="Left Trench To Alliance").add()
    auto_action.FollowPathPlannerPath(path_name="Left Trench To Alliance Center").add()

    startShooting()

    wait(1.5)

    stow_intake()

