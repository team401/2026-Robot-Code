// TypeScript replication of FieldConstants.java
// Original file copyright notice:
// https://github.com/Mechanical-Advantage/RobotCode2026Public/blob/5db972dbede8b28aed5cf21d47887b0bba90f7a9/src/main/java/org/littletonrobotics/frc2026/FieldConstants.java
// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

// The aforementioned license can be found in the file FieldConstants_LICENSE.
// All modifications made after 16:47 ET 2026-1-28 were made by FRC Team 401 Copperhead Robotics
// Docs Written by Claude Opus 4.6

import { readFileSync } from "fs";
import { resolve, dirname } from "path";
import { fileURLToPath } from "url";
import {
  Translation2d,
  Translation3d,
  Rotation2d,
  Pose2d,
  Pose3d,
} from "@/typescript/AutoAction.js";
import { pose3d } from "./Shorthands.js";

// ---------------------------------------------------------------------------
// Geometry helpers
// ---------------------------------------------------------------------------

function translation2d(x: number, y: number): Translation2d {
  return new Translation2d({ x, y });
}

function translation3d(x: number, y: number, z: number): Translation3d {
  return new Translation3d({ x, y, z });
}

function pose2d(x: number, y: number, degrees: number): Pose2d {
  return new Pose2d({
    translation: new Translation2d({ x, y }),
    rotation: new Rotation2d({ degrees }),
  });
}

// ---------------------------------------------------------------------------
// Unit conversions
// ---------------------------------------------------------------------------

function inchesToMeters(inches: number): number {
  return inches * 0.0254;
}

// ---------------------------------------------------------------------------
// AprilTag layout loading
// ---------------------------------------------------------------------------

interface TagPose {
  translation: { x: number; y: number; z: number };
  rotation: { quaternion: { W: number; X: number; Y: number; Z: number } };
}

interface TagEntry {
  ID: number;
  pose: TagPose;
}

interface AprilTagLayout {
  tags: TagEntry[];
  field: { length: number; width: number };
}

function loadAprilTagLayout(): AprilTagLayout {
  const __filename = fileURLToPath(import.meta.url);
  const __dirname = dirname(__filename);
  const layoutPath = resolve(
    __dirname,
    "../../../../src/main/deploy/apriltags/2026-rebuilt-andymark.json"
  );
  const raw = readFileSync(layoutPath, "utf-8");
  return JSON.parse(raw) as AprilTagLayout;
}

const layout: AprilTagLayout = loadAprilTagLayout();

function getTagPose(id: number): TagPose {
  const tag = layout.tags.find((t) => t.ID === id);
  if (!tag) {
    throw new Error(`AprilTag with ID ${id} not found in layout`);
  }
  return tag.pose;
}

/**
 * Convert a quaternion (W, X, Y, Z) to a yaw angle in degrees.
 * For 2D field poses we only care about rotation around Z.
 */
function quaternionToYawDegrees(q: {
  W: number;
  X: number;
  Y: number;
  Z: number;
}): number {
  const radians = Math.atan2(2.0 * (q.W * q.Z + q.X * q.Y), 1.0 - 2.0 * (q.Y * q.Y + q.Z * q.Z));
  return radians * (180.0 / Math.PI);
}

function tagPoseToPose2d(tagPose: TagPose): Pose2d {
  return pose2d(
    tagPose.translation.x,
    tagPose.translation.y,
    quaternionToYawDegrees(tagPose.rotation.quaternion)
  );
}

// ---------------------------------------------------------------------------
// FieldConstants
// ---------------------------------------------------------------------------

/**
 * Contains information for location of field elements and other useful reference points.
 *
 * NOTE: All constants are defined relative to the field coordinate system, and from the
 * perspective of the blue alliance station.
 */
export namespace FieldConstants {
  // AprilTag related constants
  export const aprilTagCount = layout.tags.length;
  export const aprilTagWidth = inchesToMeters(6.5);

  // Field dimensions
  export const fieldLength = layout.field.length;
  export const fieldWidth = layout.field.width;

  // -------------------------------------------------------------------------
  // LinesVertical
  // -------------------------------------------------------------------------

  /**
   * Officially defined and relevant vertical lines found on the field (defined by X-axis offset)
   */
  export namespace LinesVertical {
    export const center = fieldLength / 2.0;

    export const starting = getTagPose(26).translation.x;

    export const allianceZone = starting;

    export function hubCenter() {
      return getTagPose(26).translation.x + Hub.width / 2.0;
    }

    export const neutralZoneNear = center - inchesToMeters(120);

    export const neutralZoneFar = center + inchesToMeters(120);

    export function oppHubCenter() {
      return getTagPose(4).translation.x + Hub.width / 2.0;
    }

    export const oppAllianceZone = getTagPose(10).translation.x;
  }

  // -------------------------------------------------------------------------
  // LinesHorizontal
  // -------------------------------------------------------------------------

  /**
   * Officially defined and relevant horizontal lines found on the field (defined by Y-axis offset)
   *
   * NOTE: The field element start and end are always left to right from the perspective of the
   * alliance station
   */
  export namespace LinesHorizontal {
    export const center = fieldWidth / 2.0;

    // Right of hub
    export function rightBumpStart() {
      return Hub.nearRightCorner().y!;
    }

    export function rightBumpEnd() {
      return rightBumpStart() - RightBump.width;
    }

    export function rightTrenchOpenStart() {
      return rightBumpEnd() - inchesToMeters(12.0);
    }

    export const rightTrenchOpenEnd = 0;

    // Left of hub
    export function leftBumpEnd() {
      return Hub.nearLeftCorner().y!;
    }

    export function leftBumpStart() {
      return leftBumpEnd() + LeftBump.width;
    }

    export function leftTrenchOpenEnd() {
      return leftBumpStart() + inchesToMeters(12.0);
    }

    export const leftTrenchOpenStart = fieldWidth;
  }

  // -------------------------------------------------------------------------
  // Hub
  // -------------------------------------------------------------------------

  /** Hub related constants */
  export namespace Hub {
    // Dimensions
    export const width = inchesToMeters(47.0);
    export const height = inchesToMeters(72.0); // includes the catcher at the top
    export const innerWidth = inchesToMeters(41.7);
    export const innerHeight = inchesToMeters(56.5);

    // Relevant reference points on alliance side
    export function topCenterPoint(): Translation3d {
      return translation3d(
        getTagPose(26).translation.x + width / 2.0,
        fieldWidth / 2.0,
        height
      );
    }

    export function innerCenterPoint(): Translation3d {
      return translation3d(
        getTagPose(26).translation.x + width / 2.0,
        fieldWidth / 2.0,
        innerHeight
      );
    }

    export function nearLeftCorner(): Translation2d {
      return translation2d(
        topCenterPoint().x! - width / 2.0,
        fieldWidth / 2.0 + width / 2.0
      );
    }

    export function nearRightCorner(): Translation2d {
      return translation2d(
        topCenterPoint().x! - width / 2.0,
        fieldWidth / 2.0 - width / 2.0
      );
    }

    export function farLeftCorner(): Translation2d {
      return translation2d(
        topCenterPoint().x! + width / 2.0,
        fieldWidth / 2.0 + width / 2.0
      );
    }

    export function farRightCorner(): Translation2d {
      return translation2d(
        topCenterPoint().x! + width / 2.0,
        fieldWidth / 2.0 - width / 2.0
      );
    }

    // Relevant reference points on the opposite side
    export function oppTopCenterPoint(): Translation3d {
      return translation3d(
        getTagPose(4).translation.x + width / 2.0,
        fieldWidth / 2.0,
        height
      );
    }

    export function oppInnerCenterPoint(): Translation3d {
      return translation3d(
        getTagPose(4).translation.x + width / 2.0,
        fieldWidth / 2.0,
        innerHeight
      );
    }

    export function oppNearLeftCorner(): Translation2d {
      return translation2d(
        oppTopCenterPoint().x! - width / 2.0,
        fieldWidth / 2.0 + width / 2.0
      );
    }

    export function oppNearRightCorner(): Translation2d {
      return translation2d(
        oppTopCenterPoint().x! - width / 2.0,
        fieldWidth / 2.0 - width / 2.0
      );
    }

    export function oppFarLeftCorner(): Translation2d {
      return translation2d(
        oppTopCenterPoint().x! + width / 2.0,
        fieldWidth / 2.0 + width / 2.0
      );
    }

    export function oppFarRightCorner(): Translation2d {
      return translation2d(
        oppTopCenterPoint().x! + width / 2.0,
        fieldWidth / 2.0 - width / 2.0
      );
    }

    // Hub faces
    export function nearFace(): Pose2d {
      return tagPoseToPose2d(getTagPose(26));
    }

    export function farFace(): Pose2d {
      return tagPoseToPose2d(getTagPose(20));
    }

    export function rightFace(): Pose2d {
      return tagPoseToPose2d(getTagPose(18));
    }

    export function leftFace(): Pose2d {
      return tagPoseToPose2d(getTagPose(21));
    }
  }

  // -------------------------------------------------------------------------
  // LeftBump
  // -------------------------------------------------------------------------

  /** Left Bump related constants */
  export namespace LeftBump {
    // Dimensions
    export const width = inchesToMeters(73.0);
    export const height = inchesToMeters(6.513);
    export const depth = inchesToMeters(44.4);

    // Relevant reference points on alliance side
    export function nearLeftCorner(): Translation2d {
      return translation2d(
        LinesVertical.hubCenter() - width / 2,
        inchesToMeters(255)
      );
    }

    export function nearRightCorner(): Translation2d {
      return Hub.nearLeftCorner();
    }

    export function farLeftCorner(): Translation2d {
      return translation2d(
        LinesVertical.hubCenter() + width / 2,
        inchesToMeters(255)
      );
    }

    export function farRightCorner(): Translation2d {
      return Hub.farLeftCorner();
    }

    // Relevant reference points on opposing side
    export function oppNearLeftCorner(): Translation2d {
      return translation2d(
        LinesVertical.hubCenter() - width / 2,
        inchesToMeters(255)
      );
    }

    export function oppNearRightCorner(): Translation2d {
      return Hub.oppNearLeftCorner();
    }

    export function oppFarLeftCorner(): Translation2d {
      return translation2d(
        LinesVertical.hubCenter() + width / 2,
        inchesToMeters(255)
      );
    }

    export function oppFarRightCorner(): Translation2d {
      return Hub.oppFarLeftCorner();
    }
  }

  // -------------------------------------------------------------------------
  // RightBump
  // -------------------------------------------------------------------------

  /** Right Bump related constants */
  export namespace RightBump {
    // Dimensions
    export const width = inchesToMeters(73.0);
    export const height = inchesToMeters(6.513);
    export const depth = inchesToMeters(44.4);

    // Relevant reference points on alliance side
    export function nearLeftCorner(): Translation2d {
      return translation2d(
        LinesVertical.hubCenter() + width / 2,
        inchesToMeters(255)
      );
    }

    export function nearRightCorner(): Translation2d {
      return Hub.nearLeftCorner();
    }

    export function farLeftCorner(): Translation2d {
      return translation2d(
        LinesVertical.hubCenter() - width / 2,
        inchesToMeters(255)
      );
    }

    export function farRightCorner(): Translation2d {
      return Hub.farLeftCorner();
    }

    // Relevant reference points on opposing side
    export function oppNearLeftCorner(): Translation2d {
      return translation2d(
        LinesVertical.hubCenter() + width / 2,
        inchesToMeters(255)
      );
    }

    export function oppNearRightCorner(): Translation2d {
      return Hub.oppNearLeftCorner();
    }

    export function oppFarLeftCorner(): Translation2d {
      return translation2d(
        LinesVertical.hubCenter() - width / 2,
        inchesToMeters(255)
      );
    }

    export function oppFarRightCorner(): Translation2d {
      return Hub.oppFarLeftCorner();
    }
  }

  // -------------------------------------------------------------------------
  // LeftTrench
  // -------------------------------------------------------------------------

  /** Left Trench related constants */
  export namespace LeftTrench {
    // Dimensions
    export const width = inchesToMeters(65.65);
    export const depth = inchesToMeters(47.0);
    export const height = inchesToMeters(40.25);
    export const openingWidth = inchesToMeters(50.34);
    export const openingHeight = inchesToMeters(22.25);

    // Relevant reference points on alliance side
    export function openingTopLeft(): Translation3d {
      return translation3d(
        LinesVertical.hubCenter(),
        fieldWidth,
        openingHeight
      );
    }

    export function openingTopRight(): Translation3d {
      return translation3d(
        LinesVertical.hubCenter(),
        fieldWidth - openingWidth,
        openingHeight
      );
    }

    export function openingFloorCenter(): Pose3d {
      return pose3d({
            x: LinesVertical.hubCenter(),
            y: fieldWidth - openingWidth / 2,
            z: 0
      });
    }

    // Relevant reference points on opposing side
    export function oppOpeningTopLeft(): Translation3d {
      return translation3d(
        LinesVertical.oppHubCenter(),
        fieldWidth,
        openingHeight
      );
    }

    export function oppOpeningTopRight(): Translation3d {
      return translation3d(
        LinesVertical.oppHubCenter(),
        fieldWidth - openingWidth,
        openingHeight
      );
    }
  }

  // -------------------------------------------------------------------------
  // RightTrench
  // -------------------------------------------------------------------------

  /** Right Trench related constants */
  export namespace RightTrench {
    // Dimensions
    export const width = inchesToMeters(65.65);
    export const depth = inchesToMeters(47.0);
    export const height = inchesToMeters(40.25);
    export const openingWidth = inchesToMeters(50.34);
    export const openingHeight = inchesToMeters(22.25);

    // Relevant reference points on alliance side
    export function openingTopLeft(): Translation3d {
      return translation3d(
        LinesVertical.hubCenter(),
        openingWidth,
        openingHeight
      );
    }

    export function openingTopRight(): Translation3d {
      return translation3d(LinesVertical.hubCenter(), 0, openingHeight);
    }

    export function openingFloorCenter(): Pose3d {
      return pose3d({
            x: LinesVertical.hubCenter(),
            y: openingWidth / 2,
            z: 0
      });
    }

    // Relevant reference points on opposing side
    export function oppOpeningTopLeft(): Translation3d {
      return translation3d(
        LinesVertical.oppHubCenter(),
        openingWidth,
        openingHeight
      );
    }

    export function oppOpeningTopRight(): Translation3d {
      return translation3d(LinesVertical.oppHubCenter(), 0, openingHeight);
    }
  }

  // -------------------------------------------------------------------------
  // Tower
  // -------------------------------------------------------------------------

  /** Tower related constants */
  export namespace Tower {
    // Dimensions
    export const width = inchesToMeters(49.25);
    export const depth = inchesToMeters(45.0);
    export const height = inchesToMeters(78.25);
    export const innerOpeningWidth = inchesToMeters(32.25);
    export const frontFaceX = inchesToMeters(43.51);

    export const uprightHeight = inchesToMeters(72.1);

    // Rung heights from the floor
    export const lowRungHeight = inchesToMeters(27.0);
    export const midRungHeight = inchesToMeters(45.0);
    export const highRungHeight = inchesToMeters(63.0);

    // Relevant reference points on alliance side
    export function centerPoint(): Translation2d {
      return translation2d(frontFaceX, getTagPose(31).translation.y);
    }

    export function leftUpright(): Translation2d {
      return translation2d(
        frontFaceX,
        getTagPose(31).translation.y +
          innerOpeningWidth / 2 +
          inchesToMeters(0.75)
      );
    }

    export function rightUpright(): Translation2d {
      return translation2d(
        frontFaceX,
        getTagPose(31).translation.y -
          innerOpeningWidth / 2 -
          inchesToMeters(0.75)
      );
    }

    // Relevant reference points on opposing side
    export function oppCenterPoint(): Translation2d {
      return translation2d(
        fieldLength - frontFaceX,
        getTagPose(15).translation.y
      );
    }

    export function oppLeftUpright(): Translation2d {
      return translation2d(
        fieldLength - frontFaceX,
        getTagPose(15).translation.y +
          innerOpeningWidth / 2 +
          inchesToMeters(0.75)
      );
    }

    export function oppRightUpright(): Translation2d {
      return translation2d(
        fieldLength - frontFaceX,
        getTagPose(15).translation.y -
          innerOpeningWidth / 2 -
          inchesToMeters(0.75)
      );
    }
  }

  // -------------------------------------------------------------------------
  // Depot
  // -------------------------------------------------------------------------

  /** Depot related constants */
  export namespace Depot {
    // Dimensions
    export const width = inchesToMeters(42.0);
    export const depth = inchesToMeters(27.0);
    export const height = inchesToMeters(1.125);
    export const distanceFromCenterY = inchesToMeters(75.93);

    // Relevant reference points on alliance side
    export function depotCenter(): Translation3d {
      return translation3d(
        depth,
        fieldWidth / 2 + distanceFromCenterY,
        height
      );
    }

    export function leftCorner(): Translation3d {
      return translation3d(
        depth,
        fieldWidth / 2 + distanceFromCenterY + width / 2,
        height
      );
    }

    export function rightCorner(): Translation3d {
      return translation3d(
        depth,
        fieldWidth / 2 + distanceFromCenterY - width / 2,
        height
      );
    }
  }

  // -------------------------------------------------------------------------
  // Outpost
  // -------------------------------------------------------------------------

  /** Outpost related constants */
  export namespace Outpost {
    // Dimensions
    export const width = inchesToMeters(31.8);
    export const openingDistanceFromFloor = inchesToMeters(28.1);
    export const height = inchesToMeters(7.0);

    // Relevant reference points on alliance side
    export function centerPoint(): Translation2d {
      return translation2d(0, getTagPose(29).translation.y);
    }
  }

  export namespace Alliance {
    export const center: Pose2d = pose2d(
      LinesVertical.allianceZone / 2.0,
      LinesHorizontal.center,
      0
    );
  }

  export namespace OppAlliance {
    export const center: Pose2d = pose2d(
      LinesVertical.oppAllianceZone + (fieldLength - LinesVertical.oppAllianceZone) / 2.0,
      LinesHorizontal.center,
      0
    );
  }
}
