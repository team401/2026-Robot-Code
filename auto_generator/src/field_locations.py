"""
Python equivalent of FieldLocations.ts.

Contains information for location of field elements and other useful
reference points. All constants are defined relative to the field
coordinate system, and from the perspective of the blue alliance station.

Original Java source:
  https://github.com/Mechanical-Advantage/RobotCode2026Public
  Copyright (c) 2025-2026 Littleton Robotics
  License: MIT (see FieldConstants_LICENSE)
  Modifications after 16:47 ET 2026-1-28 by FRC Team 401 Copperhead Robotics
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any, Dict, List

from .auto_action import (
    Pose2d,
    Pose3d,
    Rotation2d,
    Rotation3d,
    Translation2d,
    Translation3d,
)
from .shorthands import pose3d as _make_pose3d


# ---------------------------------------------------------------------------
# Geometry helpers (local to this module)
# ---------------------------------------------------------------------------


def _translation2d(x: float, y: float) -> Translation2d:
    return Translation2d(x=x, y=y)


def _translation3d(x: float, y: float, z: float) -> Translation3d:
    return Translation3d(x=x, y=y, z=z)


def _pose2d(x: float, y: float, degrees: float) -> Pose2d:
    return Pose2d(
        translation=Translation2d(x=x, y=y),
        rotation=Rotation2d(degrees=degrees),
    )


# ---------------------------------------------------------------------------
# Unit conversions
# ---------------------------------------------------------------------------


def _inches_to_meters(inches: float) -> float:
    return inches * 0.0254


# ---------------------------------------------------------------------------
# AprilTag layout loading
# ---------------------------------------------------------------------------

_LAYOUT_PATH = (
    Path(__file__).resolve().parent.parent.parent
    / "src"
    / "main"
    / "deploy"
    / "apriltags"
    / "2026-rebuilt-andymark.json"
)


def _load_april_tag_layout() -> Dict[str, Any]:
    with open(_LAYOUT_PATH, "r") as f:
        return json.load(f)


_layout = _load_april_tag_layout()


def _get_tag_pose(tag_id: int) -> Dict[str, Any]:
    for tag in _layout["tags"]:
        if tag["ID"] == tag_id:
            return tag["pose"]
    raise ValueError(f"AprilTag with ID {tag_id} not found in layout")


def _quaternion_to_yaw_degrees(q: Dict[str, float]) -> float:
    """Convert a quaternion (W, X, Y, Z) to a yaw angle in degrees."""
    radians = math.atan2(
        2.0 * (q["W"] * q["Z"] + q["X"] * q["Y"]),
        1.0 - 2.0 * (q["Y"] ** 2 + q["Z"] ** 2),
    )
    return math.degrees(radians)


def _tag_pose_to_pose2d(tag_pose: Dict[str, Any]) -> Pose2d:
    return _pose2d(
        tag_pose["translation"]["x"],
        tag_pose["translation"]["y"],
        _quaternion_to_yaw_degrees(tag_pose["rotation"]["quaternion"]),
    )


# ---------------------------------------------------------------------------
# FieldConstants
# ---------------------------------------------------------------------------


class FieldConstants:
    """Top-level namespace for all field constants."""

    # AprilTag related constants
    april_tag_count = len(_layout["tags"])
    april_tag_width = _inches_to_meters(6.5)

    # Field dimensions
    field_length: float = _layout["field"]["length"]
    field_width: float = _layout["field"]["width"]

    # -------------------------------------------------------------------------
    # LinesVertical
    # -------------------------------------------------------------------------

    class LinesVertical:
        center = FieldConstants.field_length / 2.0 if False else 0.0  # set below

        starting = _get_tag_pose(26)["translation"]["x"]

        alliance_zone = starting

        neutral_zone_near = 0.0  # set below
        neutral_zone_far = 0.0  # set below

        opp_alliance_zone = _get_tag_pose(10)["translation"]["x"]

        @staticmethod
        def hub_center() -> float:
            return _get_tag_pose(26)["translation"]["x"] + FieldConstants.Hub.width / 2.0

        @staticmethod
        def opp_hub_center() -> float:
            return _get_tag_pose(4)["translation"]["x"] + FieldConstants.Hub.width / 2.0

    # -------------------------------------------------------------------------
    # LinesHorizontal
    # -------------------------------------------------------------------------

    class LinesHorizontal:
        center = FieldConstants.field_width / 2.0 if False else 0.0  # set below

        left_trench_open_start = 0.0  # set below

        right_trench_open_end = 0.0

        @staticmethod
        def right_bump_start() -> float:
            return FieldConstants.Hub.near_right_corner().y

        @staticmethod
        def right_bump_end() -> float:
            return FieldConstants.LinesHorizontal.right_bump_start() - FieldConstants.RightBump.width

        @staticmethod
        def right_trench_open_start() -> float:
            return FieldConstants.LinesHorizontal.right_bump_end() - _inches_to_meters(12.0)

        @staticmethod
        def left_bump_end() -> float:
            return FieldConstants.Hub.near_left_corner().y

        @staticmethod
        def left_bump_start() -> float:
            return FieldConstants.LinesHorizontal.left_bump_end() + FieldConstants.LeftBump.width

        @staticmethod
        def left_trench_open_end() -> float:
            return FieldConstants.LinesHorizontal.left_bump_start() + _inches_to_meters(12.0)

    # -------------------------------------------------------------------------
    # Hub
    # -------------------------------------------------------------------------

    class Hub:
        width = _inches_to_meters(47.0)
        height = _inches_to_meters(72.0)
        inner_width = _inches_to_meters(41.7)
        inner_height = _inches_to_meters(56.5)

        @staticmethod
        def top_center_point() -> Translation3d:
            return _translation3d(
                _get_tag_pose(26)["translation"]["x"] + FieldConstants.Hub.width / 2.0,
                FieldConstants.field_width / 2.0,
                FieldConstants.Hub.height,
            )

        @staticmethod
        def inner_center_point() -> Translation3d:
            return _translation3d(
                _get_tag_pose(26)["translation"]["x"] + FieldConstants.Hub.width / 2.0,
                FieldConstants.field_width / 2.0,
                FieldConstants.Hub.inner_height,
            )

        @staticmethod
        def near_left_corner() -> Translation2d:
            tcp = FieldConstants.Hub.top_center_point()
            return _translation2d(
                tcp.x - FieldConstants.Hub.width / 2.0,
                FieldConstants.field_width / 2.0 + FieldConstants.Hub.width / 2.0,
            )

        @staticmethod
        def near_right_corner() -> Translation2d:
            tcp = FieldConstants.Hub.top_center_point()
            return _translation2d(
                tcp.x - FieldConstants.Hub.width / 2.0,
                FieldConstants.field_width / 2.0 - FieldConstants.Hub.width / 2.0,
            )

        @staticmethod
        def far_left_corner() -> Translation2d:
            tcp = FieldConstants.Hub.top_center_point()
            return _translation2d(
                tcp.x + FieldConstants.Hub.width / 2.0,
                FieldConstants.field_width / 2.0 + FieldConstants.Hub.width / 2.0,
            )

        @staticmethod
        def far_right_corner() -> Translation2d:
            tcp = FieldConstants.Hub.top_center_point()
            return _translation2d(
                tcp.x + FieldConstants.Hub.width / 2.0,
                FieldConstants.field_width / 2.0 - FieldConstants.Hub.width / 2.0,
            )

        @staticmethod
        def opp_top_center_point() -> Translation3d:
            return _translation3d(
                _get_tag_pose(4)["translation"]["x"] + FieldConstants.Hub.width / 2.0,
                FieldConstants.field_width / 2.0,
                FieldConstants.Hub.height,
            )

        @staticmethod
        def opp_inner_center_point() -> Translation3d:
            return _translation3d(
                _get_tag_pose(4)["translation"]["x"] + FieldConstants.Hub.width / 2.0,
                FieldConstants.field_width / 2.0,
                FieldConstants.Hub.inner_height,
            )

        @staticmethod
        def opp_near_left_corner() -> Translation2d:
            tcp = FieldConstants.Hub.opp_top_center_point()
            return _translation2d(
                tcp.x - FieldConstants.Hub.width / 2.0,
                FieldConstants.field_width / 2.0 + FieldConstants.Hub.width / 2.0,
            )

        @staticmethod
        def opp_near_right_corner() -> Translation2d:
            tcp = FieldConstants.Hub.opp_top_center_point()
            return _translation2d(
                tcp.x - FieldConstants.Hub.width / 2.0,
                FieldConstants.field_width / 2.0 - FieldConstants.Hub.width / 2.0,
            )

        @staticmethod
        def opp_far_left_corner() -> Translation2d:
            tcp = FieldConstants.Hub.opp_top_center_point()
            return _translation2d(
                tcp.x + FieldConstants.Hub.width / 2.0,
                FieldConstants.field_width / 2.0 + FieldConstants.Hub.width / 2.0,
            )

        @staticmethod
        def opp_far_right_corner() -> Translation2d:
            tcp = FieldConstants.Hub.opp_top_center_point()
            return _translation2d(
                tcp.x + FieldConstants.Hub.width / 2.0,
                FieldConstants.field_width / 2.0 - FieldConstants.Hub.width / 2.0,
            )

        # Hub faces
        @staticmethod
        def near_face() -> Pose2d:
            return _tag_pose_to_pose2d(_get_tag_pose(26))

        @staticmethod
        def far_face() -> Pose2d:
            return _tag_pose_to_pose2d(_get_tag_pose(20))

        @staticmethod
        def right_face() -> Pose2d:
            return _tag_pose_to_pose2d(_get_tag_pose(18))

        @staticmethod
        def left_face() -> Pose2d:
            return _tag_pose_to_pose2d(_get_tag_pose(21))

    # -------------------------------------------------------------------------
    # LeftBump
    # -------------------------------------------------------------------------

    class LeftBump:
        width = _inches_to_meters(73.0)
        height = _inches_to_meters(6.513)
        depth = _inches_to_meters(44.4)

        @staticmethod
        def near_left_corner() -> Translation2d:
            return _translation2d(
                FieldConstants.LinesVertical.hub_center() - FieldConstants.LeftBump.width / 2,
                _inches_to_meters(255),
            )

        @staticmethod
        def near_right_corner() -> Translation2d:
            return FieldConstants.Hub.near_left_corner()

        @staticmethod
        def far_left_corner() -> Translation2d:
            return _translation2d(
                FieldConstants.LinesVertical.hub_center() + FieldConstants.LeftBump.width / 2,
                _inches_to_meters(255),
            )

        @staticmethod
        def far_right_corner() -> Translation2d:
            return FieldConstants.Hub.far_left_corner()

        @staticmethod
        def opp_near_left_corner() -> Translation2d:
            return _translation2d(
                FieldConstants.LinesVertical.hub_center() - FieldConstants.LeftBump.width / 2,
                _inches_to_meters(255),
            )

        @staticmethod
        def opp_near_right_corner() -> Translation2d:
            return FieldConstants.Hub.opp_near_left_corner()

        @staticmethod
        def opp_far_left_corner() -> Translation2d:
            return _translation2d(
                FieldConstants.LinesVertical.hub_center() + FieldConstants.LeftBump.width / 2,
                _inches_to_meters(255),
            )

        @staticmethod
        def opp_far_right_corner() -> Translation2d:
            return FieldConstants.Hub.opp_far_left_corner()

    # -------------------------------------------------------------------------
    # RightBump
    # -------------------------------------------------------------------------

    class RightBump:
        width = _inches_to_meters(73.0)
        height = _inches_to_meters(6.513)
        depth = _inches_to_meters(44.4)

        @staticmethod
        def near_left_corner() -> Translation2d:
            return _translation2d(
                FieldConstants.LinesVertical.hub_center() + FieldConstants.RightBump.width / 2,
                _inches_to_meters(255),
            )

        @staticmethod
        def near_right_corner() -> Translation2d:
            return FieldConstants.Hub.near_left_corner()

        @staticmethod
        def far_left_corner() -> Translation2d:
            return _translation2d(
                FieldConstants.LinesVertical.hub_center() - FieldConstants.RightBump.width / 2,
                _inches_to_meters(255),
            )

        @staticmethod
        def far_right_corner() -> Translation2d:
            return FieldConstants.Hub.far_left_corner()

        @staticmethod
        def opp_near_left_corner() -> Translation2d:
            return _translation2d(
                FieldConstants.LinesVertical.hub_center() + FieldConstants.RightBump.width / 2,
                _inches_to_meters(255),
            )

        @staticmethod
        def opp_near_right_corner() -> Translation2d:
            return FieldConstants.Hub.opp_near_left_corner()

        @staticmethod
        def opp_far_left_corner() -> Translation2d:
            return _translation2d(
                FieldConstants.LinesVertical.hub_center() - FieldConstants.RightBump.width / 2,
                _inches_to_meters(255),
            )

        @staticmethod
        def opp_far_right_corner() -> Translation2d:
            return FieldConstants.Hub.opp_far_left_corner()

    # -------------------------------------------------------------------------
    # LeftTrench
    # -------------------------------------------------------------------------

    class LeftTrench:
        width = _inches_to_meters(65.65)
        depth = _inches_to_meters(47.0)
        height = _inches_to_meters(40.25)
        opening_width = _inches_to_meters(50.34)
        opening_height = _inches_to_meters(22.25)

        @staticmethod
        def opening_top_left() -> Translation3d:
            return _translation3d(
                FieldConstants.LinesVertical.hub_center(),
                FieldConstants.field_width,
                FieldConstants.LeftTrench.opening_height,
            )

        @staticmethod
        def opening_top_right() -> Translation3d:
            return _translation3d(
                FieldConstants.LinesVertical.hub_center(),
                FieldConstants.field_width - FieldConstants.LeftTrench.opening_width,
                FieldConstants.LeftTrench.opening_height,
            )

        @staticmethod
        def opening_floor_center() -> Pose3d:
            return _make_pose3d(
                x=FieldConstants.LinesVertical.hub_center(),
                y=FieldConstants.field_width - FieldConstants.LeftTrench.opening_width / 2,
                z=0.0,
            )

        @staticmethod
        def opp_opening_top_left() -> Translation3d:
            return _translation3d(
                FieldConstants.LinesVertical.opp_hub_center(),
                FieldConstants.field_width,
                FieldConstants.LeftTrench.opening_height,
            )

        @staticmethod
        def opp_opening_top_right() -> Translation3d:
            return _translation3d(
                FieldConstants.LinesVertical.opp_hub_center(),
                FieldConstants.field_width - FieldConstants.LeftTrench.opening_width,
                FieldConstants.LeftTrench.opening_height,
            )

    # -------------------------------------------------------------------------
    # RightTrench
    # -------------------------------------------------------------------------

    class RightTrench:
        width = _inches_to_meters(65.65)
        depth = _inches_to_meters(47.0)
        height = _inches_to_meters(40.25)
        opening_width = _inches_to_meters(50.34)
        opening_height = _inches_to_meters(22.25)

        @staticmethod
        def opening_top_left() -> Translation3d:
            return _translation3d(
                FieldConstants.LinesVertical.hub_center(),
                FieldConstants.RightTrench.opening_width,
                FieldConstants.RightTrench.opening_height,
            )

        @staticmethod
        def opening_top_right() -> Translation3d:
            return _translation3d(
                FieldConstants.LinesVertical.hub_center(),
                0,
                FieldConstants.RightTrench.opening_height,
            )

        @staticmethod
        def opening_floor_center() -> Pose3d:
            return _make_pose3d(
                x=FieldConstants.LinesVertical.hub_center(),
                y=FieldConstants.RightTrench.opening_width / 2,
                z=0.0,
            )

        @staticmethod
        def opp_opening_top_left() -> Translation3d:
            return _translation3d(
                FieldConstants.LinesVertical.opp_hub_center(),
                FieldConstants.RightTrench.opening_width,
                FieldConstants.RightTrench.opening_height,
            )

        @staticmethod
        def opp_opening_top_right() -> Translation3d:
            return _translation3d(
                FieldConstants.LinesVertical.opp_hub_center(),
                0,
                FieldConstants.RightTrench.opening_height,
            )

    # -------------------------------------------------------------------------
    # Tower
    # -------------------------------------------------------------------------

    class Tower:
        width = _inches_to_meters(49.25)
        depth = _inches_to_meters(45.0)
        height = _inches_to_meters(78.25)
        inner_opening_width = _inches_to_meters(32.25)
        front_face_x = _inches_to_meters(43.51)

        upright_height = _inches_to_meters(72.1)

        low_rung_height = _inches_to_meters(27.0)
        mid_rung_height = _inches_to_meters(45.0)
        high_rung_height = _inches_to_meters(63.0)

        @staticmethod
        def center_point() -> Translation2d:
            return _translation2d(
                FieldConstants.Tower.front_face_x,
                _get_tag_pose(31)["translation"]["y"],
            )

        @staticmethod
        def left_upright() -> Translation2d:
            return _translation2d(
                FieldConstants.Tower.front_face_x,
                _get_tag_pose(31)["translation"]["y"]
                + FieldConstants.Tower.inner_opening_width / 2
                + _inches_to_meters(0.75),
            )

        @staticmethod
        def right_upright() -> Translation2d:
            return _translation2d(
                FieldConstants.Tower.front_face_x,
                _get_tag_pose(31)["translation"]["y"]
                - FieldConstants.Tower.inner_opening_width / 2
                - _inches_to_meters(0.75),
            )

        @staticmethod
        def opp_center_point() -> Translation2d:
            return _translation2d(
                FieldConstants.field_length - FieldConstants.Tower.front_face_x,
                _get_tag_pose(15)["translation"]["y"],
            )

        @staticmethod
        def opp_left_upright() -> Translation2d:
            return _translation2d(
                FieldConstants.field_length - FieldConstants.Tower.front_face_x,
                _get_tag_pose(15)["translation"]["y"]
                + FieldConstants.Tower.inner_opening_width / 2
                + _inches_to_meters(0.75),
            )

        @staticmethod
        def opp_right_upright() -> Translation2d:
            return _translation2d(
                FieldConstants.field_length - FieldConstants.Tower.front_face_x,
                _get_tag_pose(15)["translation"]["y"]
                - FieldConstants.Tower.inner_opening_width / 2
                - _inches_to_meters(0.75),
            )

    # -------------------------------------------------------------------------
    # Depot
    # -------------------------------------------------------------------------

    class Depot:
        width = _inches_to_meters(42.0)
        depth = _inches_to_meters(27.0)
        height = _inches_to_meters(1.125)
        distance_from_center_y = _inches_to_meters(75.93)

        @staticmethod
        def depot_center() -> Translation3d:
            fw = FieldConstants.field_width
            return _translation3d(
                FieldConstants.Depot.depth,
                fw / 2 + FieldConstants.Depot.distance_from_center_y,
                FieldConstants.Depot.height,
            )

        @staticmethod
        def left_corner() -> Translation3d:
            fw = FieldConstants.field_width
            return _translation3d(
                FieldConstants.Depot.depth,
                fw / 2 + FieldConstants.Depot.distance_from_center_y + FieldConstants.Depot.width / 2,
                FieldConstants.Depot.height,
            )

        @staticmethod
        def right_corner() -> Translation3d:
            fw = FieldConstants.field_width
            return _translation3d(
                FieldConstants.Depot.depth,
                fw / 2 + FieldConstants.Depot.distance_from_center_y - FieldConstants.Depot.width / 2,
                FieldConstants.Depot.height,
            )

    # -------------------------------------------------------------------------
    # Outpost
    # -------------------------------------------------------------------------

    class Outpost:
        width = _inches_to_meters(31.8)
        opening_distance_from_floor = _inches_to_meters(28.1)
        height = _inches_to_meters(7.0)

        @staticmethod
        def center_point() -> Translation2d:
            return _translation2d(0, _get_tag_pose(29)["translation"]["y"])

    # -------------------------------------------------------------------------
    # Alliance / OppAlliance
    # -------------------------------------------------------------------------

    class Alliance:
        pass  # populated below

    class OppAlliance:
        pass  # populated below


    class Center:
        @staticmethod
        def center_point() -> Translation2d:
            return _translation2d(
                FieldConstants.field_length / 2.0, FieldConstants.field_width / 2.0
            )

# ---------------------------------------------------------------------------
# Deferred initialization for values that depend on other FieldConstants members
# ---------------------------------------------------------------------------

FieldConstants.LinesVertical.center = FieldConstants.field_length / 2.0
FieldConstants.LinesVertical.neutral_zone_near = (
    FieldConstants.LinesVertical.center - _inches_to_meters(120)
)
FieldConstants.LinesVertical.neutral_zone_far = (
    FieldConstants.LinesVertical.center + _inches_to_meters(120)
)

FieldConstants.LinesHorizontal.center = FieldConstants.field_width / 2.0
FieldConstants.LinesHorizontal.left_trench_open_start = FieldConstants.field_width

FieldConstants.Alliance.center = _pose2d(
    FieldConstants.LinesVertical.alliance_zone / 2.0,
    FieldConstants.LinesHorizontal.center,
    0,
)

FieldConstants.OppAlliance.center = _pose2d(
    FieldConstants.LinesVertical.opp_alliance_zone
    + (FieldConstants.field_length - FieldConstants.LinesVertical.opp_alliance_zone) / 2.0,
    FieldConstants.LinesHorizontal.center,
    0,
)
