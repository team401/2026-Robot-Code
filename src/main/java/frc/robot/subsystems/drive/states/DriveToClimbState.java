package frc.robot.subsystems.drive.states;

import org.littletonrobotics.junction.Logger;

import coppercore.controls.state_machine.State;
import coppercore.controls.state_machine.StateMachine;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.constants.FieldLocations;
import frc.robot.subsystems.drive.DriveCoordinator;
import frc.robot.subsystems.drive.DriveCoordinator.ClimbLocations;
import frc.robot.subsystems.drive.control_methods.LinearDrive.LinearDriveCommand;

public class DriveToClimbState extends DrivingLinearPathState {

  public DriveToClimbState() {
    super("DriveToClimb");
  }

  public LinearDriveCommand getClimbApproachCommand(ClimbLocations climbLocation) {
    Pose2d targetPose =
        (climbLocation == ClimbLocations.LeftClimbLocation)
            ? FieldLocations.leftClimbLocation()
            : FieldLocations.rightClimbLocation();

    // TODO: Make this command have custom constraints, such as a slower max velocity, to ensure
    // precise and stable driving to the climb location
    return new LinearDriveCommand(targetPose);
  }

  // This should only be called right before we enter this state to set which climb location
  // we want to drive to, and it should not be called while we're already in this state, 
  // because it will not update the command until we exit and re-enter this state
  public void setClimbLocation(ClimbLocations climbLocation) {
    Logger.recordOutput("DriveCoordinator/DriveToClimb/ClimbLocation", climbLocation);
    setCommand(getClimbApproachCommand(climbLocation));
  }
  
}
