package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.JsonConstants;
import frc.robot.util.drive.AdjustableLinearPath;

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
        protected AdjustableLinearPath linearPath;

        public FollowLinearPathCommand(DriveCoordinator driveCoordinator, LinearDriveGoal linearDriveGoal) {
            this(driveCoordinator, linearDriveGoal, LinearDriveProfileConfig.fromJSON());
        }

        public FollowLinearPathCommand(DriveCoordinator driveCoordinator, LinearDriveGoal linearDriveGoal, LinearDriveProfileConfig profileConfig) {
            super(driveCoordinator);
            this.linearDriveGoal = linearDriveGoal;
            this.linearPath = new AdjustableLinearPath(
                profileConfig.linearConstraints, 
                profileConfig.angularConstraints
            );
        }

        @Override
        public void execute() {
            if (linearDriveGoal == null) {
                return;
            }

            AdjustableLinearPath.State pathState =
                linearPath.calculate(
                    JsonConstants.robotInfo.robotPeriod.in(Seconds),
                    new AdjustableLinearPath.State(
                        driveCoordinator.drive.getPose(),
                        ChassisSpeeds.fromRobotRelativeSpeeds(
                            driveCoordinator.drive.getChassisSpeeds(), driveCoordinator.drive.getPose().getRotation())),
                    linearDriveGoal.targetPose,
                    linearDriveGoal.endLinearVelocity,
                    linearDriveGoal.endAngularVelocity);
        
            driveCoordinator.drive.setGoalSpeedsBlueOrigins(pathState.speeds);

            // TODO: Change logging paths
                
            Logger.recordOutput("DriveCoordinator/goalOmegaRadPerSec",
                pathState.speeds.omegaRadiansPerSecond);
            Logger.recordOutput("DriveCoordinator/actualOmegaRadPerSec",
                driveCoordinator.drive.getChassisSpeeds().omegaRadiansPerSecond);
        }

        @Override
        public boolean isFinished() {

            var drive = driveCoordinator.drive;
            var currentPose = drive.getPose();
            var chassisSpeeds = drive.getChassisSpeeds();

            if (linearDriveGoal == null) {
                return true;
            }

            var currentDistanceToTarget = Meters.of(currentPose.getTranslation()
                    .getDistance(linearDriveGoal.targetPose.getTranslation()));
            if (!currentDistanceToTarget.isNear(
                Meters.zero(), JsonConstants.driveConstants.linearDriveMaxPositionError)) {
                return false;
            }

            var currentAngularError =
                Radians.of(
                    currentPose.getRotation()
                        .minus(linearDriveGoal.targetPose.getRotation()).getRadians());
            if (!currentAngularError.isNear(
                Radians.zero(), JsonConstants.driveConstants.linearDriveMaxAngularError)) {
                return false;
            }

            var currentLinearVelocity =
                MetersPerSecond.of(
                    Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond));
            if (!currentLinearVelocity.isNear(
                linearDriveGoal.endLinearVelocity, JsonConstants.driveConstants.linearDriveMaxLinearVelocityError)) {
                return false;
            }

            var currentAngularVelocity = RadiansPerSecond.of(chassisSpeeds.omegaRadiansPerSecond);
            if (!currentAngularVelocity.isNear(
                linearDriveGoal.endAngularVelocity, JsonConstants.driveConstants.linearDriveMaxAngularVelocityError)) {
                return false;
            }

            return true;
        }

    }

    public static Command linearDriveToPose(DriveCoordinator driveCoordinator, Pose2d targetPose) {
       return new FollowLinearPathCommand(driveCoordinator, LinearDriveGoal.toPose(targetPose));
    }

    public static Command linearDriveToGoal(DriveCoordinator driveCoordinator, LinearDriveGoal goal) {
        return new FollowLinearPathCommand(driveCoordinator, goal);
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
