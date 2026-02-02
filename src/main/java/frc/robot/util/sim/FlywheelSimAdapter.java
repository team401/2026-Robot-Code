package frc.robot.util.sim;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import coppercore.wpilib_interface.subsystems.configs.MechanismConfig;
import coppercore.wpilib_interface.subsystems.sim.BaseSimAdapter;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

/**
 * The FlywheelSimAdapter class wraps a WPILib FlywheelSim for use with CopperCore motor IOs by
 * implementing the CoppercoreSimAdapter interface.
 *
 * <p>FlywheelSimAdapter also extends BaseSimAdapter to allow for its use with DummySimAdapters.
 *
 * <p>Because FlywheelSim doesn't provide a position, this adapter integrates the velocity and
 * acceleration in its update method to provide a position estimate.
 */
public class FlywheelSimAdapter extends BaseSimAdapter {
  protected final FlywheelSim physicsSim;
  MutAngle integratedPosition = Radians.mutable(0.0);

  public FlywheelSimAdapter(MechanismConfig config, FlywheelSim flywheelSim) {
    super(config);
    this.physicsSim = flywheelSim;
  }

  @Override
  public void update(Voltage inputVoltage, double deltaTimeSeconds) {
    physicsSim.setInputVoltage(inputVoltage.in(Volts));
    physicsSim.update(deltaTimeSeconds);

    double dthetaRadians =
        physicsSim.getAngularVelocityRadPerSec() * deltaTimeSeconds
            + 0.5
                * physicsSim.getAngularAccelerationRadPerSecSq()
                * deltaTimeSeconds
                * deltaTimeSeconds;
    integratedPosition.mut_plus(Radians.of(dthetaRadians));
  }

  @Override
  public Angle getMotorPosition() {
    return getEncoderPosition().times(config.motorToEncoderRatio);
  }

  @Override
  public AngularVelocity getMotorAngularVelocity() {
    return getEncoderAngularVelocity().times(config.motorToEncoderRatio);
  }

  @Override
  public Angle getEncoderPosition() {
    return integratedPosition.times(config.encoderToMechanismRatio);
  }

  @Override
  public AngularVelocity getEncoderAngularVelocity() {
    return physicsSim.getAngularVelocity().times(config.encoderToMechanismRatio);
  }

  @Override
  public Current getCurrentDraw() {
    return Amps.of(physicsSim.getCurrentDrawAmps());
  }

  @Override
  public void setState(Angle newAngle, AngularVelocity newAngularVelocity) {
    integratedPosition.mut_replace(newAngle);
    physicsSim.setAngularVelocity(newAngularVelocity.in(RadiansPerSecond));
  }
}
