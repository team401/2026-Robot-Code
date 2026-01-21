package frc.robot.constants;

import coppercore.parameter_tools.json.JSONHandler;
import coppercore.parameter_tools.path_provider.EnvironmentHandler;
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

    var jsonHandler = new JSONHandler(environmentHandler.getEnvironmentPathProvider());

    robotInfo = jsonHandler.getObject(new RobotInfo(), "RobotInfo.json");
    featureFlags = jsonHandler.getObject(new FeatureFlags(), "FeatureFlags.json");
    drivetrainConstants =
        jsonHandler.getObject(new DrivetrainConstants(), "DrivetrainConstants.json");
    operatorConstants = jsonHandler.getObject(new OperatorConstants(), "OperatorConstants.json");
    turretConstants = jsonHandler.getObject(new TurretConstants(), "TurretConstants.json");
  }

  public static RobotInfo robotInfo;
  public static FeatureFlags featureFlags;
  public static DrivetrainConstants drivetrainConstants;
  public static OperatorConstants operatorConstants;
  public static TurretConstants turretConstants;
}
