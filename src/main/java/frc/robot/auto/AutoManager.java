package frc.robot.auto;

import coppercore.parameter_tools.json.JSONHandler;
import coppercore.parameter_tools.json.JSONSyncConfigBuilder;
import coppercore.parameter_tools.path_provider.PathProvider;
import edu.wpi.first.wpilibj.Filesystem;
import java.nio.file.Path;

public class AutoManager {
  public static JSONHandler jsonHandler;

  static {
    var jsonSyncSettings = new JSONSyncConfigBuilder();

    jsonSyncSettings.setUpPolymorphAdapter(AutoAction.class);

    var pathProvider =
        new PathProvider() {

          public static final Path DIRECTORY =
              Filesystem.getDeployDirectory().toPath().resolve("autos");

          @Override
          public String resolvePath(String file) {
            System.out.println("Finding Path for: " + file);
            var path = DIRECTORY.resolve(file);
            System.out.println("Path: " + path.toString());
            return path.toString();
          }
        };

    jsonHandler = new JSONHandler(jsonSyncSettings.build(), pathProvider);
  }

  public static void loadRoutines() {

  }

  public static AutoAction loadAuto(String autoName) {
    var auto = jsonHandler.getObject(new Auto(), autoName);
    System.out.println(auto);
    System.out.println(auto.auto);
    return auto.auto;
  }
}
