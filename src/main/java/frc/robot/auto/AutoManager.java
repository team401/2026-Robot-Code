package frc.robot.auto;

import coppercore.parameter_tools.json.JSONHandler;
import coppercore.parameter_tools.path_provider.PathProvider;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.util.json.FixedJSONSyncConfigBuilder;
import java.nio.file.Path;
import java.util.Optional;

public class AutoManager {
  public static JSONHandler jsonHandler;
  public static final String AUTOS_FILE = "Autos.json";
  public static final Path AUTOS_PATH =
      Filesystem.getDeployDirectory().toPath().resolve(AUTOS_FILE);
  public static Optional<Autos> autos = Optional.empty();

  static {
    var jsonSyncSettings = new FixedJSONSyncConfigBuilder();

    jsonSyncSettings.setUpPolymorphAdapter(AutoAction.class);

    jsonHandler =
        new JSONHandler(
            jsonSyncSettings.build(),
            new PathProvider() {
              @Override
              public String resolvePath(String file) {
                return AUTOS_PATH.toString();
              }
            });
  }

  public static void loadAutos() {
    autos = Optional.ofNullable(jsonHandler.getObject(new Autos(), AUTOS_PATH.toString()));
  }
}
