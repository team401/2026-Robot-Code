package frc.robot.constants;

import coppercore.parameter_tools.json.JSONHandler;
import coppercore.parameter_tools.json.JSONSyncConfigBuilder;
import coppercore.parameter_tools.path_provider.EnvironmentHandler;
import coppercore.wpilib_interface.controllers.Controllers;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.constants.drive.DriveConstants;
import frc.robot.constants.drive.DrivetrainConstants;

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
    featureFlags = jsonHandler.getObject(new FeatureFlags(), "FeatureFlags.json");
    driveConstants = jsonHandler.getObject(new DriveConstants(), "DriveConstants.json");
    drivetrainConstants =
        jsonHandler.getObject(new DrivetrainConstants(), "DrivetrainConstants.json");
    operatorConstants = jsonHandler.getObject(new OperatorConstants(), "OperatorConstants.json");
    indexerConstants = jsonHandler.getObject(new IndexerConstants(), "IndexerConstants.json");
    turretConstants = jsonHandler.getObject(new TurretConstants(), "TurretConstants.json");

    // Temporary manual calls to load fields after JSON load
    robotInfo.loadFieldsFromJSON();
    drivetrainConstants.finishLoadingConstants();
    controllers =
        jsonHandler.getObject(new Controllers(), operatorConstants.controllerBindingsFile);
  }

  public static RobotInfo robotInfo;
  public static FeatureFlags featureFlags;
  public static DriveConstants driveConstants;
  public static DrivetrainConstants drivetrainConstants;
  public static OperatorConstants operatorConstants;
  public static IndexerConstants indexerConstants;
  public static TurretConstants turretConstants;
  public static Controllers controllers;
}
