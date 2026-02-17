package frc.robot.subsystems.drive.states;

import coppercore.controls.state_machine.State;
import coppercore.controls.state_machine.StateMachine;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.constants.FieldLocations;
import frc.robot.subsystems.drive.DriveCoordinator;
import frc.robot.subsystems.drive.DriveCoordinator.ClimbLocations;
import frc.robot.subsystems.drive.control_methods.LinearDrive.LinearDriveCommand;

public class DriveToClimbState extends State<DriveCoordinator> {

  private ClimbLocations climbLocation =
      ClimbLocations.LeftClimbLocation; // Default to left climb location if not set

  public DriveToClimbState() {
    super("DriveToClimb");
  }

  public void setClimbLocation(ClimbLocations climbLocation) {
    this.climbLocation = climbLocation;
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

  @Override
  public void onEntry(StateMachine<DriveCoordinator> stateMachine, DriveCoordinator world) {
    world.LINEAR_DRIVE.setCommand(getClimbApproachCommand(climbLocation));
  }

  @Override
  protected void periodic(StateMachine<DriveCoordinator> stateMachine, DriveCoordinator world) {
    world.setControlMethod(world.LINEAR_DRIVE);
    if (world.LINEAR_DRIVE.isFinished()) {
      finish();
    }
  }
}
