package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.control_methods.LinearDrive.LinearDriveCommand;

public class DriveCoordinatorCommands extends Command {
    
    protected DriveCoordinator driveCoordinator;

    public DriveCoordinatorCommands(DriveCoordinator driveCoordinator) {
        this.driveCoordinator = driveCoordinator;
    }

    private static class JoystickDriveCommand extends DriveCoordinatorCommands {

        public JoystickDriveCommand(DriveCoordinator driveCoordinator) {
            super(driveCoordinator);
        }

        @Override
        public void execute() {
            driveCoordinator.setControlMethod(driveCoordinator.JOYSTICK_DRIVE);
        }

    }

    private static class StopDriveCommand extends DriveCoordinatorCommands {

        public StopDriveCommand(DriveCoordinator driveCoordinator) {
            super(driveCoordinator);
        }

        @Override
        public void execute() {
            driveCoordinator.setControlMethod(driveCoordinator.DISABLED_DRIVE);
        }

    }

    private static class FollowLinearPathCommand extends DriveCoordinatorCommands {
        private LinearDriveCommand linearDriveCommand;

        public FollowLinearPathCommand(DriveCoordinator driveCoordinator, LinearDriveCommand linearDriveCommand) {
            super(driveCoordinator);
            this.linearDriveCommand = linearDriveCommand;
        }

        @Override
        public void initialize() {
            driveCoordinator.followLinearDriveCommand(linearDriveCommand);
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
       return followLinearDriveCommand(driveCoordinator, new LinearDriveCommand(targetPose));
    }

    public static Command followLinearDriveCommand(DriveCoordinator driveCoordinator, LinearDriveCommand linearDriveCommand) {
        return new FollowLinearPathCommand(driveCoordinator, linearDriveCommand);
    }

    public static Command joystickDrive(DriveCoordinator driveCoordinator) {
        return new JoystickDriveCommand(driveCoordinator);
    }

    public static Command stopDrive(DriveCoordinator driveCoordinator) {
        return new StopDriveCommand(driveCoordinator);
    }
}
