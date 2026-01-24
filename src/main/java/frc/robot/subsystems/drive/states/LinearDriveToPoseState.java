package frc.robot.subsystems.drive.states;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import com.ctre.phoenix6.swerve.utility.LinearPath;
import coppercore.controls.state_machine.State;
import coppercore.controls.state_machine.StateMachine;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

// Copilot used to help write this class

public class LinearDriveToPoseState extends State<Drive> {

  private Pose2d targetPose;

  private TrapezoidProfile.Constraints linearConstraints;
  private TrapezoidProfile.Constraints angularConstraints;

  private LinearPath linearPath;

  private double timeElapsed = 0.0;

  public void setTargetPose(Pose2d targetPose) {
    this.targetPose = targetPose;
    reconstructLinearPath();
  }

  protected void setLinearConstraints(TrapezoidProfile.Constraints constraints) {
    this.linearConstraints = constraints;
    reconstructLinearPath();
  }

  protected void setAngularConstraints(TrapezoidProfile.Constraints constraints) {
    this.angularConstraints = constraints;
    reconstructLinearPath();
  }

  protected void setConstraints(
      TrapezoidProfile.Constraints linearConstraints,
      TrapezoidProfile.Constraints angularConstraints) {
    this.linearConstraints = linearConstraints;
    this.angularConstraints = angularConstraints;
    reconstructLinearPath();
  }

  protected void reconstructLinearPath() {
    linearPath = new LinearPath(linearConstraints, angularConstraints);
    // Reset time elapsed when reconstructing the path
    timeElapsed = 0.0;
  }

  public LinearDriveToPoseState() {
    super("LinearDrive");

    setConstraints(
        new TrapezoidProfile.Constraints(
            JsonConstants.drivetrainConstants.maxLinearSpeed.in(MetersPerSecond),
            JsonConstants.drivetrainConstants.maxLinearAcceleration.in(MetersPerSecondPerSecond)),
        new TrapezoidProfile.Constraints(
            JsonConstants.drivetrainConstants.maxAngularSpeed.in(RadiansPerSecond),
            JsonConstants.drivetrainConstants.maxAngularAcceleration.in(
                RadiansPerSecondPerSecond)));
  }

  @Override
  public void onEntry(StateMachine<Drive> stateMachine, Drive world) {
    reconstructLinearPath();
  }

  @Override
  protected void periodic(StateMachine<Drive> stateMachine, Drive world) {

    LinearPath.State pathState =
        linearPath.calculate(
            0.02,
            new LinearPath.State(
                world.getPose(),
                ChassisSpeeds.fromRobotRelativeSpeeds(
                    world.getChassisSpeeds(), world.getPose().getRotation())),
            targetPose);

    world.setGoalSpeedsBlueOrigins(pathState.speeds);

    timeElapsed += 0.02; // Assuming periodic is called every 20ms

    Logger.recordOutput(
        "Drive/LinearDriveToPoseState/goalOmegaRadPerSec", pathState.speeds.omegaRadiansPerSecond);
    Logger.recordOutput(
        "Drive/LinearDriveToPoseState/actualOmegaRadPerSec",
        world.getChassisSpeeds().omegaRadiansPerSecond);

    if (world.getPose().getTranslation().getDistance(targetPose.getTranslation()) < 0.02
        && Math.abs(world.getPose().getRotation().minus(targetPose.getRotation()).getRadians())
            < 0.05) {
      finish();
      return;
    }
  }
}
