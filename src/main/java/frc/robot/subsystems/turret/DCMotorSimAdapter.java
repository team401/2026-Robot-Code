package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Amps;

import coppercore.wpilib_interface.subsystems.configs.MechanismConfig;
import coppercore.wpilib_interface.subsystems.sim.BasePositionSimAdapter;
import edu.wpi.first.units.measure.Current;
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

  // TODO: implement PositionSimAdapter methods
  public Current getCurrentDrawAmps() {
    return Amps.of(dcMotorSim.getCurrentDrawAmps());
  }
}
