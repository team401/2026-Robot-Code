package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.drive.control_methods.LinearDrive.LinearDriveCommand;

public class DriveCoordinatorCommands extends Command {
    
    // Add logging

    protected DriveCoordinator driveCoordinator;

    public DriveCoordinatorCommands(DriveCoordinator driveCoordinator) {
        this.driveCoordinator = driveCoordinator;
    }

    private static class StopDriveCommand extends DriveCoordinatorCommands {

        public StopDriveCommand(DriveCoordinator driveCoordinator) {
            super(driveCoordinator);
        }

        @Override
        public void execute() {
            driveCoordinator.drive.stop();
        }

    }

    public static record LinearDriveGoal(
        Pose2d targetPose,
        LinearVelocity endLinearVelocity,
        AngularVelocity endAngularVelocity) {

        public static LinearDriveGoal toPose(Pose2d targetPose) {
            return new LinearDriveGoal(targetPose, MetersPerSecond.zero(), RadiansPerSecond.zero());
        }

        public static LinearDriveGoal toPoseWithEndVelocities(
            Pose2d targetPose, LinearVelocity endLinearVelocity, AngularVelocity endAngularVelocity) {
            return new LinearDriveGoal(targetPose, endLinearVelocity, endAngularVelocity);
        }
    }

    public static record LinearDriveProfileConfig(
        TrapezoidProfile.Constraints linearConstraints,
        TrapezoidProfile.Constraints angularConstraints) {

        public static LinearDriveProfileConfig fromJSON() {
        return new LinearDriveProfileConfig(
            new TrapezoidProfile.Constraints(
                JsonConstants.driveConstants.linearDriveProfileMaxLinearVelocity.in(MetersPerSecond),
                JsonConstants.driveConstants.linearDriveProfileMaxLinearAcceleration.in(
                    MetersPerSecondPerSecond)),
            new TrapezoidProfile.Constraints(
                JsonConstants.driveConstants.linearDriveProfileMaxAngularVelocity.in(
                    RadiansPerSecond),
                JsonConstants.driveConstants.linearDriveProfileMaxAngularAcceleration.in(
                    RadiansPerSecondPerSecond)));
        }
    }

    private static class FollowLinearPathCommand extends DriveCoordinatorCommands {
        private LinearDriveGoal linearDriveGoal;
        private LinearDriveProfileConfig profileConfig;

        public FollowLinearPathCommand(DriveCoordinator driveCoordinator, LinearDriveGoal linearDriveGoal) {
            this(driveCoordinator, linearDriveGoal, LinearDriveProfileConfig.fromJSON());
        }

        public FollowLinearPathCommand(DriveCoordinator driveCoordinator, LinearDriveGoal linearDriveGoal, LinearDriveProfileConfig profileConfig) {
            super(driveCoordinator);
            this.linearDriveGoal = linearDriveGoal;
            this.profileConfig = profileConfig;
        }

        @Override
        public void initialize() {
            // driveCoordinator.followLinearDriveCommand(linearDriveCommand);
        }

        @Override
        public void execute() {
            driveCoordinator.setControlMethod(driveCoordinator.LINEAR_DRIVE);
        }

        @Override
        public boolean isFinished() {
            return driveCoordinator.LINEAR_DRIVE.isFinished();
        }

    }

    public static Command linearDriveToPose(DriveCoordinator driveCoordinator, Pose2d targetPose) {
       return new FollowLinearPathCommand(driveCoordinator, LinearDriveGoal.toPose(targetPose));
    }

    public static Command linearDriveWithConfig(DriveCoordinator driveCoordinator, LinearDriveGoal linearDriveGoal, LinearDriveProfileConfig profileConfig) {
        return new FollowLinearPathCommand(driveCoordinator, linearDriveGoal, profileConfig);
    }

    public static Command joystickDrive(DriveCoordinator driveCoordinator) {
        return driveCoordinator.joystickCommand;
    }

    public static Command stopDrive(DriveCoordinator driveCoordinator) {
        return new StopDriveCommand(driveCoordinator);
    }
}
