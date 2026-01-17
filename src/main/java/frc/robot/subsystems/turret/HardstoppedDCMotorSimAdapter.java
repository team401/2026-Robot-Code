package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import coppercore.wpilib_interface.subsystems.configs.MechanismConfig;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * The HardstoppedDCMotorSimAdapter class extends {@link DCMotorSimAdapter} to add hardstops (hard
 * limits on the range of motion of the mechanism).
 *
 * <p>At the end of update, if the mechanism's position is outside of its allowed range of motion,
 * it will be snapped back to the closest hardstop its velocity will be zeroed.
 */
public class HardstoppedDCMotorSimAdapter extends DCMotorSimAdapter {
  private final Angle lowerLimitMechanismPosition;
  private final Angle upperLimitMechanismPosition;

  /**
   * Creates a new HardstoppedDcMotorSimAdapter, given a MechanismConfig, DCMotorSim to wrap, and
   * lower and upper bounds for movement.
   *
   * <p>Keep in mind that the hardstop positions are given in terms of the mechanism. That means
   * that if your upper limit is 1 rotation, motor:encoder ratio is 30, and your encoder:mechanism
   * ratio is 1, the motor would be allowed to reach a position of 30 before it is stopped.
   *
   * @param config The MechanismConfig, used to find the ratios between the motor, encoder, and
   *     mechanism
   * @param dcMotorSim The DCMotorSim physics simulation object to wrap with this adapter
   * @param lowerLimitMechanismPosition The <em>mechanism position</em> of the lower hardstop. If
   *     the mechanism position is below this value after update, it will be clamped upward to this
   *     value.
   * @param upperLimitMechanismPosition The <em>mechanism position</em> of the upper hardstop. If
   *     the mechanism position is above this value after update, it will be clamped downward to
   *     this value.
   */
  public HardstoppedDCMotorSimAdapter(
      MechanismConfig config,
      DCMotorSim dcMotorSim,
      Angle lowerLimitMechanismPosition,
      Angle upperLimitMechanismPosition) {
    super(config, dcMotorSim);

    this.lowerLimitMechanismPosition = lowerLimitMechanismPosition;
    this.upperLimitMechanismPosition = upperLimitMechanismPosition;
  }

  @Override
  public void update(Voltage motorAppliedOutput, double deltaTimeSeconds) {
    super.update(motorAppliedOutput, deltaTimeSeconds);

    Angle mechanismPos = getEncoderPosition().div(config.encoderToMechanismRatio);

    if (mechanismPos.lt(lowerLimitMechanismPosition)) {
      setState(
          lowerLimitMechanismPosition
              .times(config.encoderToMechanismRatio)
              .times(config.motorToEncoderRatio),
          RadiansPerSecond.zero());
    } else if (mechanismPos.gt(upperLimitMechanismPosition)) {
      setState(
          upperLimitMechanismPosition
              .times(config.encoderToMechanismRatio)
              .times(config.motorToEncoderRatio),
          RadiansPerSecond.zero());
    }
  }
}
