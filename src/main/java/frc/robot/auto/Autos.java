package frc.robot.auto;

import java.util.HashMap;

public class Autos {

  public HashMap<String, AutoAction> autos = new HashMap<>();

  public AutoAction getAuto(String name) {
    return autos.get(name);
  }
}
