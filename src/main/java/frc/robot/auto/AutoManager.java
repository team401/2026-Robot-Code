package frc.robot.auto;

import coppercore.parameter_tools.json.JSONHandler;
import coppercore.parameter_tools.json.JSONSyncConfigBuilder;
import coppercore.parameter_tools.path_provider.PathProvider;
import edu.wpi.first.wpilibj.Filesystem;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.stream.Stream;

public class AutoManager {
  public static JSONHandler jsonHandler;
  public static final String AUTO_PATH = "autos";
  public static final String ROUTINE_PATH = "routines";
  public static final Path DIRECTORY = Filesystem.getDeployDirectory().toPath().resolve(AUTO_PATH);

  static {
    var jsonSyncSettings = new JSONSyncConfigBuilder();

    jsonSyncSettings.setUpPolymorphAdapter(AutoAction.class);

    var pathProvider =
        new PathProvider() {

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

  public static void registerRoutine(String routineName, AutoAction auto) {
    RoutineRegistry.register(routineName, auto);
  }

  public static void loadRoutine(String routineName) {
    var routine = jsonHandler.getObject(new Routine(), ROUTINE_PATH + "/" + routineName);
    var name = routine.name;
    if (name == null) {
      if (routineName.endsWith(".json")) {
        name = routineName.substring(0, routineName.length() - 5);
      } else {
        name = routineName;
      }
    }
    registerRoutine(name, routine.routine);
  }

  public static void loadRoutines(String... routineNames) {
    for (var routineName : routineNames) {
      loadRoutine(routineName);
    }
  }

  public static String[] getAvailableRoutines() {
    return RoutineRegistry.getRegisteredRoutines();
  }

  public static Auto loadAuto(String autoName) {
    var auto = jsonHandler.getObject(new Auto(), autoName);
    return auto;
  }

  private static String[] getJSONFilesInDirectory(String directory) {
    try (Stream<Path> stream = Files.list(DIRECTORY.resolve(directory))) {
      return stream
          .filter(Files::isRegularFile)
          .filter(path -> path.getFileName().toString().endsWith(".json"))
          .map(path -> path.getFileName().toString())
          .toArray(String[]::new);
    } catch (Exception e) {
      e.printStackTrace();
      return new String[0];
    }
  }

  public static String[] getRoutineFileNames() {
    return getJSONFilesInDirectory(ROUTINE_PATH);
  }

  public static String[] getAutoFileNames() {
    return getJSONFilesInDirectory("");
  }

  public static void loadAllRoutines() {
    loadRoutines(getRoutineFileNames());
  }
}
