package frc.robot.auto;

import java.util.HashMap;

public class RoutineRegistry {
  public static HashMap<String, AutoAction> routines = new HashMap<>();

  public static void register(String name, AutoAction action) {
    routines.put(name, action);
  }

  public static AutoAction getRoutine(String name) {
    return routines.get(name);
  }

  public static String[] getRegisteredRoutines() {
    return routines.keySet().toArray(String[]::new);
  }
}
