package frc.robot.subsystems.drive;

import coppercore.wpilib_interface.DriveWithJoysticks;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.FieldLocations;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.drive.DriveCoordinatorCommands.LinearDriveGoal;
import frc.robot.subsystems.drive.DriveCoordinatorCommands.LinearDriveProfileConfig;
import frc.robot.subsystems.drive.control_methods.DriveControlMethod;
import frc.robot.subsystems.drive.control_methods.LinearDrive;
import frc.robot.util.LoggedTunablePIDGains;
import frc.robot.util.TestModeManager;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import org.littletonrobotics.junction.Logger;

// Copilot used to help write this class

// States currently are more so what is drive trying to do
// And Control Methods are more so how drive is trying to do it

public class DriveCoordinator extends SubsystemBase {

  public enum ClimbLocations {
    LeftClimbLocation,
    RightClimbLocation
  }

  public Drive drive;

  public LinearDrive LINEAR_DRIVE;

  private DriveControlMethod currentControlMethod;

  public void setControlMethod(DriveControlMethod controlMethod) {
    // Just here currently to make java happy
  }

  public void initializeJoyStickDriveControl(DriveWithJoysticks command) {
    this.joystickCommand = command;
    // Maybe temporary
    this.defaultCommand = joystickCommand;
  }

  public void initializeControlMethods(Drive drive) {
    LINEAR_DRIVE = new LinearDrive(drive, "DriveCoordinator");
  }

  // Test Mode
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

  public Command currentCommand;

  protected Command defaultCommand;
  protected DriveWithJoysticks joystickCommand;

  /**
   * The active command is always has the periodic run currently before it checks if it is finished.
   * @return
   */
  protected Command getCurrentActiveCommand() {
    if (currentCommand != null) {
      return currentCommand;
    }
    return defaultCommand;
  }

  // TODO: Determine if we want to have a default command, and if it should be stopDrive or joystickCommand
  /**
   * Sets the default command for the DriveCoordinator. The default command is used when there is no current active command.
   * If the current active command is null when this method is called, the new default command will be initialized immediately.
   * If the current active command is the same as the current default command, it will be ended and the new default command will be initialized.
   * If the current default command is the same as the new default command, this method will do nothing.
   * @param command The new default command to set. If null, it will default to the joystick command.
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
   * Sets the current command for the DriveCoordinator.
   * If the new command is different from the current active command, it will end the current command
   * (if it exists) and initialize the new command (if it's not null).
   * This method ensures that only one command is active at a time and that the default command is used
   * when no other command is set.
   * 
   * If the command#isFinished() method returns true, this will call command#end(false) otherwise
   * it will call command#end(true). This allows for proper cleanup of the command based on whether it finished successfully
   * or was interrupted by another command.
   * 
   * Also the command will automatically be ended when it says it is finished.
   * @param command The new command to set as the current command. If null, the default command will be used.
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

  private void finishCurrentCommandIfFinished() {
    if (currentCommand != null && currentCommand.isFinished()) {
      currentCommand.end(true);
      currentCommand = null;
    }else{
      if (defaultCommand != null && defaultCommand.isFinished()) {
        defaultCommand.end(true);
        
      }
    }
  }

  public DriveCoordinator(Drive drive) {
    this.drive = drive;

    this.defaultCommand = DriveCoordinatorCommands.stopDrive(this);
    this.currentCommand = null;
    // drive.setDriveGains(JsonConstants.driveConstants.driveGains);
    // drive.setSteerGains(JsonConstants.driveConstants.steerGains);

    initializeControlMethods(drive);
  }

  public void linearDriveToPose(Pose2d pose) {
    setCurrentCommand(DriveCoordinatorCommands.linearDriveToPose(this, pose));
  }

  public void linearDriveWithConfig(LinearDriveGoal command, LinearDriveProfileConfig linearDriveConfig) {
    setCurrentCommand(DriveCoordinatorCommands.linearDriveWithConfig(this, command, linearDriveConfig));
  }

  public Command getDriveToClimbCommand(ClimbLocations climbLocation) {
    Pose2d targetPose =
        (climbLocation == ClimbLocations.LeftClimbLocation)
            ? FieldLocations.leftClimbLocation()
            : FieldLocations.rightClimbLocation();

    var linearDriveConfig = LinearDriveProfileConfig.fromJSON();

    return DriveCoordinatorCommands.linearDriveWithConfig(
      this, 
      LinearDriveGoal.toPose(targetPose),
      linearDriveConfig
    );
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
    

    // There should always be a current control method, but just in case, we can check for null
    // before calling periodic on it
    if (currentControlMethod != null) {
      currentControlMethod.periodic();
    }
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
