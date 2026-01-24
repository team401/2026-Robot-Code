package frc.robot.constants;

import com.ctre.phoenix6.CANBus;

public class RobotInfo {

  // JSON Only Fields (For initializing values from JSON files)

  private final String canivoreBusName = "canivore";

  private final String logFilePath = "./logs/robot_log.hoot";

  // Normal Fields

  // Fields loaded after JSON

  public CANBus kCANBus;

  // @AfterJSONLoad
  public void loadFieldsFromJSON() {
    kCANBus = new CANBus(canivoreBusName, logFilePath);
  }
}
