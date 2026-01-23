package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import coppercore.wpilib_interface.subsystems.configs.MechanismConfig;
import coppercore.wpilib_interface.subsystems.motors.MotorIO;
import coppercore.wpilib_interface.subsystems.motors.talonfx.MotorIOTalonFX;
import edu.wpi.first.units.measure.AngularVelocity;

// I helped copilot autocomplete and chat gpt 5 write this file
public class HopperSubsystem {
  MotorIO motorIO;

  public static MotorIO getTalonFXMotorIO() {
    return MotorIOTalonFX.newLeader(MechanismConfig.builder().build(), new TalonFXConfiguration());
  }

  public static MotorIO getTalonFXMotorPositionSimIO() {
    return MotorIOTalonFXPositionSim.newLeader(
        MechanismConfig.builder().build(), new TalonFXConfiguration());
  }

  public HopperSubsystem(MotorIO motorIO) {
    this.motorIO = motorIO;
  }

  public void runHopperAtSpeed(AngularVelocity speed) {
    motorIO.controlToVelocityUnprofiled(speed);
  }

  public void stopHopper() {
    motorIO.controlToVelocityUnprofiled(RadiansPerSecond.of(0.0));
  }
}
