package frc.robot.constants;

import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.DigitalInputsConfigs;
import com.ctre.phoenix6.signals.S1CloseStateValue;
import com.ctre.phoenix6.signals.S1FloatStateValue;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.io.dio_switch.DigitalInputIOCANdi.CANdiSignal;

public class RobotInfo {
  // TODO: Start a comprehensive list of all CAN ids on the robot (either in google sheets, a
  // markdown file here, or one CANConstants.{java, json} file)
  public final String canivoreBusName = "canivore";

  // Pose taken from KrayonCAD
  public final Transform3d robotToShooter =
      new Transform3d(
          Units.inchesToMeters(4.175),
          Units.inchesToMeters(-2.088),
          Units.inchesToMeters(17.0),
          new Rotation3d());

  public final Integer homingSwitchCANdiID = 14; // TODO: Real id

  public final CANdiSignal homingSwitchSignal = CANdiSignal.S1;

  public final CANdiConfiguration buildHomingSwitchConfig() {
    return new CANdiConfiguration()
        .withDigitalInputs(
            new DigitalInputsConfigs()
                .withS1CloseState(S1CloseStateValue.CloseWhenHigh)
                .withS1FloatState(S1FloatStateValue.PullLow));
  }
}
