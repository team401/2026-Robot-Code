package frc.robot.subsystems.drive;

import coppercore.wpilib_interface.DriveWithJoysticks;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.FieldLocations;
import frc.robot.constants.JsonConstants;
import frc.robot.util.LoggedTunablePIDGains;
import frc.robot.util.TestModeManager;
import org.littletonrobotics.junction.Logger;

// TODO: Add lots of logging to this class and the commands

public class DriveCoordinator extends SubsystemBase {

  // Testing Mode Fields
  final TestModeManager<TestMode> testModeManager = new TestModeManager<>("Drive", TestMode.class);

  LoggedTunablePIDGains steerGains;
  LoggedTunablePIDGains driveGains;

  public void initializeTestMode() {
    steerGains =
        new LoggedTunablePIDGains(
            "DriveCoordinatorTunables/SteerGains",
            JsonConstants.driveConstants.steerGains.asArray());
    driveGains =
        new LoggedTunablePIDGains(
            "DriveCoordinatorTunables/DriveGains",
            JsonConstants.driveConstants.driveGains.asArray());
  }

  // Fields for normal operation

  public enum ClimbLocations {
    LeftClimbLocation,
    RightClimbLocation
  }

  public Drive drive;

  public Command currentCommand;

  protected Command defaultCommand;
  protected DriveWithJoysticks joystickCommand;

  public void setDriveWithJoysticksCommand(DriveWithJoysticks command) {
    this.joystickCommand = command;
    // Maybe temporary
    this.defaultCommand = joystickCommand;
  }

  /**
   * The active command is always has the periodic run currently before it checks if it is finished.
   *
   * @return
   */
  protected Command getCurrentActiveCommand() {
    if (currentCommand != null) {
      return currentCommand;
    }
    return defaultCommand;
  }

  // TODO: Determine if we want to have a default command, and if it should be stopDrive or
  // joystickCommand
  /**
   * Sets the default command for the DriveCoordinator. The default command is used when there is no
   * current active command. If the current active command is null when this method is called, the
   * new default command will be initialized immediately. If the current active command is the same
   * as the current default command, it will be ended and the new default command will be
   * initialized. If the current default command is the same as the new default command, this method
   * will do nothing.
   *
   * @param command The new default command to set. If null, it will default to the joystick
   *     command.
   */
  public void setDefaultCommand(Command command) {
    var newDefaultCommand = (command == null) ? joystickCommand : command;
    if (defaultCommand == newDefaultCommand) {
      return;
    }
    var activeCommand = getCurrentActiveCommand();
    if (defaultCommand == activeCommand) {
      activeCommand.end(activeCommand.isFinished());
      if (newDefaultCommand != null) {
        newDefaultCommand.initialize();
      }
    }
    defaultCommand = newDefaultCommand;
  }

  /**
   * Sets the current command for the DriveCoordinator. If the new command is different from the
   * current active command, it will end the current command (if it exists) and initialize the new
   * command (if it's not null). This method ensures that only one command is active at a time and
   * that the default command is used when no other command is set.
   *
   * <p>If the command#isFinished() method returns true, this will call command#end(false) otherwise
   * it will call command#end(true). This allows for proper cleanup of the command based on whether
   * it finished successfully or was interrupted by another command.
   *
   * <p>Also the command will automatically be ended when it says it is finished.
   *
   * @param command The new command to set as the current command. If null, the default command will
   *     be used.
   */
  public void setCurrentCommand(Command command) {
    var activeCommand = getCurrentActiveCommand();
    var nextCommand = command == null ? defaultCommand : command;
    if (activeCommand != nextCommand) {
      if (activeCommand != null) {
        activeCommand.end(activeCommand.isFinished());
      }
      currentCommand = command;
      if (currentCommand != null) {
        currentCommand.initialize();
      }
    }
  }

  public void cancelCurrentCommand() {
    setCurrentCommand(null);
  }

  public InstantCommand createInstantCommandToSetCurrent(Command command) {
    return new InstantCommand(() -> setCurrentCommand(command));
  }

  public InstantCommand createInstantCommandToCancelCommand() {
    return new InstantCommand(this::cancelCurrentCommand);
  }

  public void forceRestartCurrentCommand() {
    var activeCommand = getCurrentActiveCommand();
    if (activeCommand != null) {
      activeCommand.end(activeCommand.isFinished());
      activeCommand.initialize();
    }
  }

  private void finishCurrentCommandIfFinished() {
    if (currentCommand != null && currentCommand.isFinished()) {
      currentCommand.end(true);
      currentCommand = null;
    } else {
      if (defaultCommand != null && defaultCommand.isFinished()) {
        defaultCommand.end(true);
      }
    }
  }

  public DriveCoordinator(Drive drive) {
    this.drive = drive;

    this.defaultCommand = DriveCoordinatorCommands.stopDrive(this);
    this.currentCommand = null;
    this.joystickCommand = null;

    // drive.setDriveGains(JsonConstants.driveConstants.driveGains);
    // drive.setSteerGains(JsonConstants.driveConstants.steerGains);
  }

  public void autoPilotToPose(Pose2d pose) {
    setCurrentCommand(DriveCoordinatorCommands.autoPilotToPoseCommand(this, pose));
  }

  public Command getDriveToClimbCommand(ClimbLocations climbLocation) {
    Pose2d targetPose =
        (climbLocation == ClimbLocations.LeftClimbLocation)
            ? FieldLocations.leftClimbLocation()
            : FieldLocations.rightClimbLocation();

    // TODO: Enhance this to not just use the default settings

    return DriveCoordinatorCommands.autoPilotToPoseCommand(this, targetPose);
  }

  public void driveToClimbLocation(ClimbLocations climbLocation) {
    setCurrentCommand(getDriveToClimbCommand(climbLocation));
  }

  @Override
  public void periodic() {

    if (testModeManager.isInTestMode()) {
      testPeriodic();
    }

    // Maybe decide if we want to check if it is finished before or after executing.
    // And if we check before executing, do we want it to be able to go through multiple commands
    // in one periodic if they are all finished, or just one command per periodic?
    var activeCommand = getCurrentActiveCommand();
    if (activeCommand != null) {
      activeCommand.execute();
      finishCurrentCommandIfFinished();
    }

    // While yes the active command technically could be different from what is logged here
    // But this is meant to log the command that we ran this periodic, not necessarily the command
    // that is active at the end of this periodic.
    Logger.recordOutput(
        "DriveCoordinator/CurrentCommand",
        activeCommand == null ? "None" : activeCommand.getName());
  }

  public void testPeriodic() {
    switch (testModeManager.getTestMode()) {
      case DriveGainsTuning:
        steerGains.ifChanged(hashCode(), drive::setSteerGains);
        driveGains.ifChanged(hashCode(), drive::setDriveGains);
        break;
      default:
        break;
    }
  }
}
