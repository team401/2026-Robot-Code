package frc.robot.constants;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.DigitalInputsConfigs;
import com.ctre.phoenix6.signals.S1CloseStateValue;
import com.ctre.phoenix6.signals.S1FloatStateValue;
import coppercore.parameter_tools.json.annotations.AfterJsonLoad;
import coppercore.parameter_tools.json.annotations.JSONExclude;
import coppercore.wpilib_interface.subsystems.motors.talonfx.MotorIOTalonFX.SignalRefreshRates;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;
import frc.robot.util.io.dio_switch.DigitalInputIOCANdi.CANdiSignal;

public class RobotInfo {

  // JSON Only Fields (For initializing values from JSON files)

  // Private here, everyone else should use the CANBus object below
  private String canivoreBusName = "canivore";

  private String logFilePath = "./logs/robot_log.hoot";

  public Mass robotMass = Kilograms.of(74.088);
  public MomentOfInertia robotMOI = KilogramSquareMeters.of(6.883);
  public Double wheelCof = 1.2;

  public Time robotPeriod = Seconds.of(0.02);

  public Boolean runAutoTesting = false;

  public double lowVoltageThreshold = 12.2;
  public double batteryAlertFilterTime = 0.05;

  // Pose taken from KrayonCAD
  public final Transform3d robotToShooter =
      new Transform3d(
          Units.inchesToMeters(4.175),
          Units.inchesToMeters(-2.088),
          Units.inchesToMeters(17.0),
          new Rotation3d());

  @JSONExclude public Transform2d robotToShooter2d;

  public final CANdiSignal homingSwitchSignal = CANdiSignal.S1;

  public final CANdiConfiguration buildHomingSwitchConfig() {
    return new CANdiConfiguration()
        .withDigitalInputs(
            new DigitalInputsConfigs()
                .withS1CloseState(S1CloseStateValue.CloseWhenHigh)
                .withS1FloatState(S1FloatStateValue.PullHigh));
  }

  /** The refresh rates that should be used for subsystems that don't effect fire control */
  public final SignalRefreshRates nonFireControllingRefreshRates =
      new SignalRefreshRates(Hertz.of(50.0), Hertz.of(20.0), Hertz.of(50.0));

  // Normal Fields

  // Fields populated by loadFieldsFromJSON

  @JSONExclude public CANBus CANBus;

  @JSONExclude public double ODOMETRY_FREQUENCY;

  @AfterJsonLoad
  public void loadFieldsFromJSON() {
    CANBus = new CANBus(canivoreBusName, logFilePath);
    ODOMETRY_FREQUENCY = CANBus.isNetworkFD() ? 250.0 : 100.0;

    robotToShooter2d =
        new Transform2d(
            robotToShooter.getX(),
            robotToShooter.getY(),
            robotToShooter.getRotation().toRotation2d());
  }
}
