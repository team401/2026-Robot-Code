package frc.robot.constants;

import com.ctre.phoenix6.CANBus;

import coppercore.parameter_tools.json.annotations.AfterJsonLoad;
import coppercore.parameter_tools.json.annotations.JSONExclude;

public class RobotInfo {

  // JSON Only Fields (For initializing values from JSON files)

  private final String canivoreBusName = "canivore";

  private final String logFilePath = "./logs/robot_log.hoot";

  // Normal Fields

  // Fields loaded after JSON

  @JSONExclude
  public CANBus CANBus;

  @AfterJsonLoad
  public void loadFieldsFromJSON() {
    CANBus = new CANBus(canivoreBusName, logFilePath);
  }
}
