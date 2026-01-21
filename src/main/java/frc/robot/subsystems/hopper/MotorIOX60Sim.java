package frc.robot.subsystems.hopper;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import coppercore.wpilib_interface.subsystems.configs.MechanismConfig;
import coppercore.wpilib_interface.subsystems.motors.talonfx.MotorIOTalonFXPositionSim;
import coppercore.wpilib_interface.subsystems.sim.PositionSimAdapter;

public class MotorIOX60Sim extends MotorIOTalonFXPositionSim {
  public MotorIOX60Sim(
      MechanismConfig config, TalonFXConfiguration talonFXConfigs, PositionSimAdapter physicsSim) {
    super(config, talonFXConfigs, physicsSim);
    talon.getSimState().setMotorType(MotorType.KrakenX60);
  }
}
