package frc.robot.subsystems.hood;

import coppercore.wpilib_interface.MonitorWithAlert.MonitorWithAlertBuilder;
import coppercore.wpilib_interface.MonitoredSubsystem;
import coppercore.wpilib_interface.subsystems.motors.MotorIO;
import coppercore.wpilib_interface.subsystems.motors.MotorInputsAutoLogged;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.constants.JsonConstants;
import org.littletonrobotics.junction.AutoLogOutputManager;

/**
 * The HoodSubsystem class defines the Hood subsystem, which controls the hardware for the hood on
 * the aimer of our shooter superstructure. It uses simple closed loop control to accomplish its
 * tasks.
 */
public class HoodSubsystem extends MonitoredSubsystem {
  private final MotorIO motor;
  private final MotorInputsAutoLogged inputs = new MotorInputsAutoLogged();

  public HoodSubsystem(MotorIO motor) {
    this.motor = motor;

    addMonitor(
        new MonitorWithAlertBuilder()
            .withName("HoodMotorDisconnected")
            .withAlertText("Hood motor disconnected")
            .withAlertType(AlertType.kError)
            .withTimeToFault(JsonConstants.hoodConstants.disconnectedDebounceTimeSeconds)
            .withLoggingEnabled(true)
            .withStickyness(false)
            .withIsStateValidSupplier(() -> inputs.connected)
            .withFaultCallback(() -> {})
            .build());

    // TODO: Add state machine for homing

    AutoLogOutputManager.addObject(this);
  }

  @Override
  public void monitoredPeriodic() {
    motor.updateInputs(inputs);
  }
}
