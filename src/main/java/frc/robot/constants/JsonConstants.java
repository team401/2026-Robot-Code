package frc.robot.constants;

import coppercore.parameter_tools.json.JSONHandler;
import coppercore.parameter_tools.json.JSONSyncConfigBuilder;
import coppercore.parameter_tools.path_provider.EnvironmentHandler;
import coppercore.wpilib_interface.controllers.Controllers;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.constants.drive.DriveConstants;
import frc.robot.constants.drive.PhysicalDriveConstants;

/**
 * JsonConstants handles loading and saving of all constants through JSON. Call `loadConstants`
 * before using constants.
 */
public class JsonConstants {
  public static EnvironmentHandler environmentHandler;

  public static void loadConstants() {
    environmentHandler =
        EnvironmentHandler.getEnvironmentHandler(
            Filesystem.getDeployDirectory().toPath().resolve("constants/config.json").toString());

    var jsonSyncSettings = new JSONSyncConfigBuilder();

    Controllers.applyControllerConfigToBuilder(jsonSyncSettings);

    var jsonHandler =
        new JSONHandler(jsonSyncSettings.build(), environmentHandler.getEnvironmentPathProvider());

    robotInfo = jsonHandler.getObject(new RobotInfo(), "RobotInfo.json");
    aprilTagConstants = jsonHandler.getObject(new AprilTagConstants(), "AprilTagConstants.json");
    featureFlags = jsonHandler.getObject(new FeatureFlags(), "FeatureFlags.json");
    canBusAssignment = jsonHandler.getObject(new CANBusAssignment(), "CANBusAssignment.json");
    driveConstants = jsonHandler.getObject(new DriveConstants(), "DriveConstants.json");

    // jsonHandler.saveObject(new VisionConstants(), "VisionConstants.json");
    visionConstants = jsonHandler.getObject(new VisionConstants(), "VisionConstants.json");

    physicalDriveConstants =
        jsonHandler.getObject(new PhysicalDriveConstants(), "PhysicalDriveConstants.json");
    operatorConstants = jsonHandler.getObject(new OperatorConstants(), "OperatorConstants.json");
    hopperConstants = jsonHandler.getObject(new HopperConstants(), "HopperConstants.json");
    indexerConstants = jsonHandler.getObject(new IndexerConstants(), "IndexerConstants.json");
    turretConstants = jsonHandler.getObject(new TurretConstants(), "TurretConstants.json");
    intakeConstants = jsonHandler.getObject(new IntakeConstants(), "IntakeConstants.json");
    shooterConstants = jsonHandler.getObject(new ShooterConstants(), "ShooterConstants.json");
    hoodConstants = jsonHandler.getObject(new HoodConstants(), "HoodConstants.json");

    if (featureFlags.useTuningServer) {
      // do not crash Robot if routes could not be added for any reason
      try {
        jsonHandler.addRoute("/hopper", hopperConstants);
        jsonHandler.addRoute("/indexer", indexerConstants);
        jsonHandler.addRoute("/turret", turretConstants);
        jsonHandler.addRoute("/drive", driveConstants);
        jsonHandler.addRoute("/hood", hoodConstants);
      } catch (Exception ex) {
        System.err.println("could not add routes for constant tuning: " + ex);
      }
    }

    controllers =
        jsonHandler.getObject(new Controllers(), operatorConstants.controllerBindingsFile);
  }

  public static RobotInfo robotInfo;
  public static AprilTagConstants aprilTagConstants;
  public static CANBusAssignment canBusAssignment;
  public static FeatureFlags featureFlags;
  public static DriveConstants driveConstants;
  public static VisionConstants visionConstants;
  public static PhysicalDriveConstants physicalDriveConstants;
  public static OperatorConstants operatorConstants;
  public static HopperConstants hopperConstants;
  public static IndexerConstants indexerConstants;
  public static TurretConstants turretConstants;
  public static IntakeConstants intakeConstants;
  public static ShooterConstants shooterConstants;
  public static HoodConstants hoodConstants;
  public static Controllers controllers;
}
