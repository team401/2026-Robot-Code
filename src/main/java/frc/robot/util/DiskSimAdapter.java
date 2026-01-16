package frc.robot.util;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import coppercore.wpilib_interface.subsystems.configs.MechanismConfig;
import coppercore.wpilib_interface.subsystems.sim.PositionSimAdapter;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class DiskSimAdapter implements PositionSimAdapter {
  private final MechanismConfig config;
  // The states of the system are [angle,
  // angular velocity]ᵀ, inputs are [voltage], and outputs are [angle, angular velocity]
  private final DiskSim diskSim;

  /**
   * Create a new ArmSimAdapter to adapt a given SingleJointedArmSim using parameters specified in a
   * given MechanismConfig.
   *
   * @param config The MechanismConfig, used to find ratios of encoder to mechanism and motor to
   *     encoder.
   * @param armSim The SingleJointedArmSim, whose position and velocity will be used to update
   *     inputs in sim.
   */
  public DiskSimAdapter(MechanismConfig config, DiskSim diskSim) {
    this.config = config;
    this.diskSim = diskSim;
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
    Angle mechanismPos = Radians.of(diskSim.getAngleRads());
    Angle encoderPos = mechanismPos.times(config.encoderToMechanismRatio);

    return encoderPos;
  }

  @Override
  public AngularVelocity getEncoderAngularVelocity() {
    double mechanismVelRadPerSec = diskSim.getVelocityRadPerSec();
    double encoderVelRadPerSec = mechanismVelRadPerSec * config.encoderToMechanismRatio;

    return RadiansPerSecond.of(encoderVelRadPerSec);
  }

  @Override
  public Current getCurrentDraw() {
    return Amps.of(diskSim.getCurrentDrawAmps());
  }

  @Override
  public void update(Voltage motorAppliedOutput, double deltaTimeSeconds) {
    diskSim.setInput(motorAppliedOutput.in(Volts));
    diskSim.update(deltaTimeSeconds);
  }

  /**
   * Manually sets the physics sim's position and velocity. This is intended to be used by the
   * DummySimAdapter to mock different positions and observe how a physics sim responds.
   *
   * <p>This method must NOT be called by normal sim code unless you know what you are doing. It
   * will instantly set the position of a mechanism with no regard for physical limitations.
   *
   * @param motorAngle A double representing the position to set, in radians
   * @param motorVelocity A double representing the velocity to set, in radians per second
   */
  protected void setState(Angle motorAngle, AngularVelocity motorVelocity) {
    diskSim.setState(motorAngle.in(Radians), motorVelocity.in(RadiansPerSecond));
  }
}
