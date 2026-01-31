package frc.robot.constants;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.CANBus;

import coppercore.parameter_tools.json.annotations.AfterJsonLoad;
import coppercore.parameter_tools.json.annotations.JSONExclude;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;

public class RobotInfo {

  // JSON Only Fields (For initializing values from JSON files)

  private final String canivoreBusName = "canivore";

  private final String logFilePath = "./logs/robot_log.hoot";

  public Mass robotMass = Kilograms.of(74.088);
  public MomentOfInertia robotMOI = KilogramSquareMeters.of(6.883);
  public Double wheelCof = 1.2;

  public Time robotPeriod = Seconds.of(0.02);

  // Normal Fields

  // Fields loaded after JSON

  @JSONExclude
  public CANBus CANBus;

  @AfterJsonLoad
  public void loadFieldsFromJSON() {
    CANBus = new CANBus(canivoreBusName, logFilePath);
  }
}
