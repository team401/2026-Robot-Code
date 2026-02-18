package frc.robot.subsystems.drive.states;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.constants.FieldLocations;
import frc.robot.subsystems.drive.DriveCoordinator.ClimbLocations;
import frc.robot.subsystems.drive.control_methods.LinearDrive.LinearDriveCommand;
import frc.robot.subsystems.drive.control_methods.LinearDrive.LinearDriveProfileConfig;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import org.littletonrobotics.junction.Logger;

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
    return new LinearDriveCommand(targetPose, MetersPerSecond.zero(), RadiansPerSecond.zero(), LinearDriveProfileConfig.fromJSON());
  }

  // This should only be called right before we enter this state to set which climb location
  // we want to drive to, and it should not be called while we're already in this state,
  // because it will not update the command until we exit and re-enter this state
  public void setClimbLocation(ClimbLocations climbLocation) {
    Logger.recordOutput("DriveCoordinator/DriveToClimb/ClimbLocation", climbLocation);
    setCommand(getClimbApproachCommand(climbLocation));
  }
}
