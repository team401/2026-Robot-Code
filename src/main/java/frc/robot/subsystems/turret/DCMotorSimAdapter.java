package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import coppercore.wpilib_interface.subsystems.configs.MechanismConfig;
import coppercore.wpilib_interface.subsystems.sim.BasePositionSimAdapter;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * The DCMotorSimAdapter class wraps a DCMotorSim to implement the PositionSimAdapter interface,
 * allowing DCMotorSim to be used with Coppercore motor IOs in simulation.
 */
public class DCMotorSimAdapter extends BasePositionSimAdapter {
  protected final DCMotorSim dcMotorSim;

  public DCMotorSimAdapter(MechanismConfig config, DCMotorSim dcMotorSim) {
    super(config);

    this.dcMotorSim = dcMotorSim;
  }

  @Override
  public Angle getMotorPosition() {
    return dcMotorSim.getAngularPosition();
  }

  @Override
  public AngularVelocity getMotorAngularVelocity() {
    return dcMotorSim.getAngularVelocity();
  }

  @Override
  public Angle getEncoderPosition() {
    return dcMotorSim.getAngularPosition().div(config.motorToEncoderRatio);
  }

  @Override
  public AngularVelocity getEncoderAngularVelocity() {
    return dcMotorSim.getAngularVelocity().div(config.motorToEncoderRatio);
  }

  @Override
  public Current getCurrentDraw() {
    return Amps.of(dcMotorSim.getCurrentDrawAmps());
  }

  @Override
  public void update(Voltage motorAppliedOutput, double deltaTimeSeconds) {
    dcMotorSim.setInput(motorAppliedOutput.in(Volts));
    dcMotorSim.update(deltaTimeSeconds);
  }

  @Override
  protected void setState(Angle motorAngle, AngularVelocity motorVelocity) {
    dcMotorSim.setState(motorAngle.in(Radians), motorVelocity.in(RadiansPerSecond));
  }
}
