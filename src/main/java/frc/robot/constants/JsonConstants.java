package frc.robot.constants;

import coppercore.parameter_tools.json.JSONHandler;
import coppercore.parameter_tools.json.JSONSyncConfigBuilder;
import coppercore.parameter_tools.path_provider.EnvironmentHandler;
import coppercore.wpilib_interface.controllers.Controllers;
import edu.wpi.first.wpilibj.Filesystem;

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
    drivetrainConstants =
        jsonHandler.getObject(new DrivetrainConstants(), "DrivetrainConstants.json");
    operatorConstants = jsonHandler.getObject(new OperatorConstants(), "OperatorConstants.json");
    indexerConstants = jsonHandler.getObject(new IndexerConstants(), "IndexerConstants.json");
    turretConstants = jsonHandler.getObject(new TurretConstants(), "TurretConstants.json");
    shooterConstants = jsonHandler.getObject(new ShooterConstants(), "ShooterConstants.json");
    shooterConstants.initializeViMap(); // TODO: Use AfterJsonLoad annotation for this instead
    hoodConstants = jsonHandler.getObject(new HoodConstants(), "HoodConstants.json");

    if (featureFlags.useTuningServer) {
      // do not crash Robot if routes could not be added for any reason
      try {
        jsonHandler.addRoute("/indexer", indexerConstants);
        jsonHandler.addRoute("/turret", turretConstants);
      } catch (Exception ex) {
        System.err.println("could not add routes for constant tuning: " + ex);
      }
    }

    controllers =
        jsonHandler.getObject(new Controllers(), operatorConstants.controllerBindingsFile);
  }

  public static RobotInfo robotInfo;
  public static AprilTagConstants aprilTagConstants;
  public static FeatureFlags featureFlags;
  public static DrivetrainConstants drivetrainConstants;
  public static OperatorConstants operatorConstants;
  public static IndexerConstants indexerConstants;
  public static TurretConstants turretConstants;
  public static ShooterConstants shooterConstants;
  public static HoodConstants hoodConstants;

  public static Controllers controllers;
}
