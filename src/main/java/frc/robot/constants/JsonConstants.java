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

    drivetrainConstants =
        jsonHandler.getObject(new DrivetrainConstants(), "DrivetrainConstants.json");
    jsonHandler.saveObject(new OperatorConstants(), "OperatorConstants.json");
    operatorConstants = jsonHandler.getObject(new OperatorConstants(), "OperatorConstants.json");
  }

  public static DrivetrainConstants drivetrainConstants;
  public static OperatorConstants operatorConstants;
}
