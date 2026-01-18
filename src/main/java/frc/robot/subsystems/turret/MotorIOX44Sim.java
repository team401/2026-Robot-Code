package frc.robot.subsystems.turret;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import coppercore.wpilib_interface.subsystems.configs.MechanismConfig;
import coppercore.wpilib_interface.subsystems.motors.talonfx.MotorIOTalonFXPositionSim;
import coppercore.wpilib_interface.subsystems.sim.PositionSimAdapter;

public class MotorIOX44Sim extends MotorIOTalonFXPositionSim {
  public MotorIOX44Sim(
      MechanismConfig config, TalonFXConfiguration talonFXConfigs, PositionSimAdapter physicsSim) {
    super(config, talonFXConfigs, physicsSim);
    talon.getSimState().setMotorType(MotorType.KrakenX44);
  }
}
