// Original file copyright notice:
// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

// The aforementioned license can be found in the file FieldConstants_LICENSE
// All modifications made after 16:47 ET 2026-1-28 were made by FRC Team 401 Copperhead Robotics

package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

// TODO: Replace "opp" references with a good way to get constants for whatever alliance we're on

/**
 * Contains information for location of field element and other useful reference points.
 *
 * <p>NOTE: All constants are defined relative to the field coordinate system, and from the
 * perspective of the blue alliance station
 *
 * <p>NOTE: Many of these constants are getter methods derived from the current value of
 * JsonConstants.aprilTagConstants.fieldType
 */
public class FieldConstants {
  // AprilTag related constants
  public static final int aprilTagCount() {
    return JsonConstants.aprilTagConstants.getTagLayout().getTags().size();
  }

  public static final double aprilTagWidth = Units.inchesToMeters(6.5);

  // Field dimensions
  public static final double fieldLength() {
    return JsonConstants.aprilTagConstants.getTagLayout().getFieldLength();
  }

  public static final double fieldWidth() {
    return JsonConstants.aprilTagConstants.getTagLayout().getFieldWidth();
  }

  /**
   * Officially defined and relevant vertical lines found on the field (defined by X-axis offset)
   */
  public static class LinesVertical {
    public static final double center() {
      return fieldLength() / 2.0;
    }

    public static final double starting() {
      return JsonConstants.aprilTagConstants.getTagLayout().getTagPose(26).get().getX();
    }

    public static final double allianceZone() {
      return starting();
    }

    public static final double hubCenter() {
      return JsonConstants.aprilTagConstants.getTagLayout().getTagPose(26).get().getX()
          + Hub.width / 2.0;
    }

    public static final double neutralZoneNear() {
      return center() - Units.inchesToMeters(120);
    }

    public static final double neutralZoneFar() {
      return center() + Units.inchesToMeters(120);
    }

    public static final double oppHubCenter() {
      return JsonConstants.aprilTagConstants.getTagLayout().getTagPose(4).get().getX()
          + Hub.width / 2.0;
    }

    public static final double oppAllianceZone() {
      return JsonConstants.aprilTagConstants.getTagLayout().getTagPose(10).get().getX();
    }
  }

  /**
   * Officially defined and relevant horizontal lines found on the field (defined by Y-axis offset)
   *
   * <p>NOTE: The field element start and end are always left to right from the perspective of the
   * alliance station
   */
  public static class LinesHorizontal {

    public static final double center() {
      return fieldWidth() / 2.0;
    }

    // Right of hub
    public static final double rightBumpStart() {
      return Hub.nearRightCorner().getY();
    }

    public static final double rightBumpEnd() {
      return rightBumpStart() - RightBump.width;
    }

    public static final double rightTrenchOpenStart() {
      return rightBumpEnd() - Units.inchesToMeters(12.0);
    }

    public static final double rightTrenchOpenEnd() {
      return 0;
    }
    ;

    // Left of hub
    public static final double leftBumpEnd() {
      return Hub.nearLeftCorner().getY();
    }

    public static final double leftBumpStart() {
      return leftBumpEnd() + LeftBump.width;
    }

    public static final double leftTrenchOpenEnd() {
      return leftBumpStart() + Units.inchesToMeters(12.0);
    }

    public static final double leftTrenchOpenStart() {
      return fieldWidth();
    }
    ;
  }

  /** Hub related constants */
  public static class Hub {

    // Dimensions
    public static final double width = Units.inchesToMeters(47.0);
    public static final double height =
        Units.inchesToMeters(72.0); // includes the catcher at the top
    public static final double innerWidth = Units.inchesToMeters(41.7);
    public static final double innerHeight = Units.inchesToMeters(56.5);

    // Relevant reference points on alliance side
    public static final Translation3d topCenterPoint() {
      return new Translation3d(
          JsonConstants.aprilTagConstants.getTagLayout().getTagPose(26).get().getX() + width / 2.0,
          fieldWidth() / 2.0,
          height);
    }

    public static final Translation3d innerCenterPoint() {
      return new Translation3d(
          JsonConstants.aprilTagConstants.getTagLayout().getTagPose(26).get().getX() + width / 2.0,
          fieldWidth() / 2.0,
          innerHeight);
    }

    public static final Translation2d nearLeftCorner() {
      return new Translation2d(
          topCenterPoint().getX() - width / 2.0, fieldWidth() / 2.0 + width / 2.0);
    }

    public static final Translation2d nearRightCorner() {
      return new Translation2d(
          topCenterPoint().getX() - width / 2.0, fieldWidth() / 2.0 - width / 2.0);
    }

    public static final Translation2d farLeftCorner() {
      return new Translation2d(
          topCenterPoint().getX() + width / 2.0, fieldWidth() / 2.0 + width / 2.0);
    }

    public static final Translation2d farRightCorner() {
      return new Translation2d(
          topCenterPoint().getX() + width / 2.0, fieldWidth() / 2.0 - width / 2.0);
    }

    // Relevant reference points on the opposite side
    public static final Translation3d oppTopCenterPoint() {
      return new Translation3d(
          JsonConstants.aprilTagConstants.getTagLayout().getTagPose(4).get().getX() + width / 2.0,
          fieldWidth() / 2.0,
          height);
    }

    public static final Translation2d oppNearLeftCorner() {
      return new Translation2d(
          oppTopCenterPoint().getX() - width / 2.0, fieldWidth() / 2.0 + width / 2.0);
    }

    public static final Translation2d oppNearRightCorner() {
      return new Translation2d(
          oppTopCenterPoint().getX() - width / 2.0, fieldWidth() / 2.0 - width / 2.0);
    }

    public static final Translation2d oppFarLeftCorner() {
      return new Translation2d(
          oppTopCenterPoint().getX() + width / 2.0, fieldWidth() / 2.0 + width / 2.0);
    }

    public static final Translation2d oppFarRightCorner() {
      return new Translation2d(
          oppTopCenterPoint().getX() + width / 2.0, fieldWidth() / 2.0 - width / 2.0);
    }

    // Hub faces
    public static final Pose2d nearFace() {
      return JsonConstants.aprilTagConstants.getTagLayout().getTagPose(26).get().toPose2d();
    }

    public static final Pose2d farFace() {
      return JsonConstants.aprilTagConstants.getTagLayout().getTagPose(20).get().toPose2d();
    }

    public static final Pose2d rightFace() {
      return JsonConstants.aprilTagConstants.getTagLayout().getTagPose(18).get().toPose2d();
    }

    public static final Pose2d leftFace() {
      return JsonConstants.aprilTagConstants.getTagLayout().getTagPose(21).get().toPose2d();
    }
  }

  /** Left Bump related constants */
  public static class LeftBump {

    // Dimensions
    public static final double width = Units.inchesToMeters(73.0);
    public static final double height = Units.inchesToMeters(6.513);
    public static final double depth = Units.inchesToMeters(44.4);

    // Relevant reference points on alliance side
    public static final Translation2d nearLeftCorner() {
      return new Translation2d(LinesVertical.hubCenter() - width / 2, Units.inchesToMeters(255));
    }

    public static final Translation2d nearRightCorner() {
      return Hub.nearLeftCorner();
    }

    public static final Translation2d farLeftCorner() {
      return new Translation2d(LinesVertical.hubCenter() + width / 2, Units.inchesToMeters(255));
    }

    public static final Translation2d farRightCorner() {
      return Hub.farLeftCorner();
    }
    ;

    // Relevant reference points on opposing side
    public static final Translation2d oppNearLeftCorner() {
      return new Translation2d(LinesVertical.hubCenter() - width / 2, Units.inchesToMeters(255));
    }

    public static final Translation2d oppNearRightCorner() {
      return Hub.oppNearLeftCorner();
    }

    public static final Translation2d oppFarLeftCorner() {
      return new Translation2d(LinesVertical.hubCenter() + width / 2, Units.inchesToMeters(255));
    }

    public static final Translation2d oppFarRightCorner() {
      return Hub.oppFarLeftCorner();
    }
    ;
  }

  /** Right Bump related constants */
  public static class RightBump {
    // Dimensions
    public static final double width = Units.inchesToMeters(73.0);
    public static final double height = Units.inchesToMeters(6.513);
    public static final double depth = Units.inchesToMeters(44.4);

    // Relevant reference points on alliance side
    public static final Translation2d nearLeftCorner() {
      return new Translation2d(LinesVertical.hubCenter() + width / 2, Units.inchesToMeters(255));
    }

    public static final Translation2d nearRightCorner() {
      return Hub.nearLeftCorner();
    }

    public static final Translation2d farLeftCorner() {
      return new Translation2d(LinesVertical.hubCenter() - width / 2, Units.inchesToMeters(255));
    }

    public static final Translation2d farRightCorner() {
      return Hub.farLeftCorner();
    }

    // Relevant reference points on opposing side
    public static final Translation2d oppNearLeftCorner() {
      return new Translation2d(LinesVertical.hubCenter() + width / 2, Units.inchesToMeters(255));
    }

    public static final Translation2d oppNearRightCorner() {
      return Hub.oppNearLeftCorner();
    }

    public static final Translation2d oppFarLeftCorner() {
      return new Translation2d(LinesVertical.hubCenter() - width / 2, Units.inchesToMeters(255));
    }

    public static final Translation2d oppFarRightCorner() {
      return Hub.oppFarLeftCorner();
    }
  }

  /** Left Trench related constants */
  public static class LeftTrench {
    // Dimensions
    public static final double width = Units.inchesToMeters(65.65);
    public static final double depth = Units.inchesToMeters(47.0);
    public static final double height = Units.inchesToMeters(40.25);
    public static final double openingWidth = Units.inchesToMeters(50.34);
    public static final double openingHeight = Units.inchesToMeters(22.25);

    // Relevant reference points on alliance side
    public static final Translation3d openingTopLeft() {
      return new Translation3d(LinesVertical.hubCenter(), fieldWidth(), openingHeight);
    }

    public static final Translation3d openingTopRight() {
      return new Translation3d(
          LinesVertical.hubCenter(), fieldWidth() - openingWidth, openingHeight);
    }

    // Relevant reference points on opposing side
    public static final Translation3d oppOpeningTopLeft() {
      return new Translation3d(LinesVertical.oppHubCenter(), fieldWidth(), openingHeight);
    }

    public static final Translation3d oppOpeningTopRight() {
      return new Translation3d(
          LinesVertical.oppHubCenter(), fieldWidth() - openingWidth, openingHeight);
    }
  }

  public static class RightTrench {

    // Dimensions
    public static final double width = Units.inchesToMeters(65.65);
    public static final double depth = Units.inchesToMeters(47.0);
    public static final double height = Units.inchesToMeters(40.25);
    public static final double openingWidth = Units.inchesToMeters(50.34);
    public static final double openingHeight = Units.inchesToMeters(22.25);

    // Relevant reference points on alliance side
    public static final Translation3d openingTopLeft() {
      return new Translation3d(LinesVertical.hubCenter(), openingWidth, openingHeight);
    }

    public static final Translation3d openingTopRight() {
      return new Translation3d(LinesVertical.hubCenter(), 0, openingHeight);
    }

    // Relevant reference points on opposing side
    public static final Translation3d oppOpeningTopLeft() {
      return new Translation3d(LinesVertical.oppHubCenter(), openingWidth, openingHeight);
    }

    public static final Translation3d oppOpeningTopRight() {
      return new Translation3d(LinesVertical.oppHubCenter(), 0, openingHeight);
    }
  }

  /** Tower related constants */
  public static class Tower {
    // Dimensions
    public static final double width = Units.inchesToMeters(49.25);
    public static final double depth = Units.inchesToMeters(45.0);
    public static final double height = Units.inchesToMeters(78.25);
    public static final double innerOpeningWidth = Units.inchesToMeters(32.250);
    public static final double frontFaceX = Units.inchesToMeters(43.51);

    public static final double uprightHeight = Units.inchesToMeters(72.1);

    // Rung heights from the floor
    public static final double lowRungHeight = Units.inchesToMeters(27.0);
    public static final double midRungHeight = Units.inchesToMeters(45.0);
    public static final double highRungHeight = Units.inchesToMeters(63.0);

    // Relevant reference points on alliance side
    public static final Translation2d centerPoint() {
      return new Translation2d(
          frontFaceX, JsonConstants.aprilTagConstants.getTagLayout().getTagPose(31).get().getY());
    }

    public static final Translation2d leftUpright() {
      return new Translation2d(
          frontFaceX,
          (JsonConstants.aprilTagConstants.getTagLayout().getTagPose(31).get().getY())
              + innerOpeningWidth / 2
              + Units.inchesToMeters(0.75));
    }

    public static final Translation2d rightUpright() {
      return new Translation2d(
          frontFaceX,
          (JsonConstants.aprilTagConstants.getTagLayout().getTagPose(31).get().getY())
              - innerOpeningWidth / 2
              - Units.inchesToMeters(0.75));
    }

    // Relevant reference points on opposing side
    public static final Translation2d oppCenterPoint() {
      return new Translation2d(
          fieldLength() - frontFaceX,
          JsonConstants.aprilTagConstants.getTagLayout().getTagPose(15).get().getY());
    }

    public static final Translation2d oppLeftUpright() {
      return new Translation2d(
          fieldLength() - frontFaceX,
          (JsonConstants.aprilTagConstants.getTagLayout().getTagPose(15).get().getY())
              + innerOpeningWidth / 2
              + Units.inchesToMeters(0.75));
    }

    public static final Translation2d oppRightUpright() {
      return new Translation2d(
          fieldLength() - frontFaceX,
          (JsonConstants.aprilTagConstants.getTagLayout().getTagPose(15).get().getY())
              - innerOpeningWidth / 2
              - Units.inchesToMeters(0.75));
    }
  }

  public static class Depot {
    // Dimensions
    public static final double width = Units.inchesToMeters(42.0);
    public static final double depth = Units.inchesToMeters(27.0);
    public static final double height = Units.inchesToMeters(1.125);
    public static final double distanceFromCenterY = Units.inchesToMeters(75.93);

    // Relevant reference points on alliance side
    public static final Translation3d depotCenter() {
      return new Translation3d(depth, (fieldWidth() / 2) + distanceFromCenterY, height);
    }

    public static final Translation3d leftCorner() {
      return new Translation3d(
          depth, (fieldWidth() / 2) + distanceFromCenterY + (width / 2), height);
    }

    public static final Translation3d rightCorner() {
      return new Translation3d(
          depth, (fieldWidth() / 2) + distanceFromCenterY - (width / 2), height);
    }
  }

  public static class Outpost {
    // Dimensions
    public static final double width = Units.inchesToMeters(31.8);
    public static final double openingDistanceFromFloor = Units.inchesToMeters(28.1);
    public static final double height = Units.inchesToMeters(7.0);

    // Relevant reference points on alliance side
    public static final Translation2d centerPoint() {
      return new Translation2d(
          0, JsonConstants.aprilTagConstants.getTagLayout().getTagPose(29).get().getY());
    }
  }
}
