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

    featureFlags = jsonHandler.getObject(new FeatureFlags(), "FeatureFlags.json");
    drivetrainConstants =
        jsonHandler.getObject(new DrivetrainConstants(), "DrivetrainConstants.json");
    operatorConstants = jsonHandler.getObject(new OperatorConstants(), "OperatorConstants.json");
    jsonHandler.saveObject(new IndexerConstants(), "IndexerConstants.json");
    indexerConstants = jsonHandler.getObject(new IndexerConstants(), "IndexerConstants.json");
  }

  public static FeatureFlags featureFlags;
  public static DrivetrainConstants drivetrainConstants;
  public static OperatorConstants operatorConstants;
  public static IndexerConstants indexerConstants;
}
