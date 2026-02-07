// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import coppercore.wpilib_interface.DriveTemplate;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.constants.JsonConstants;
import frc.robot.util.LocalADStarAK;
import frc.robot.util.PIDGains;
import frc.robot.util.littletonUtil.PoseEstimator;
import frc.robot.util.littletonUtil.PoseEstimator.TimestampedVisionUpdate;
import java.util.List;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase implements DriveTemplate {

  // PathPlanner config constants
  private final double ROBOT_MASS_KG = JsonConstants.robotInfo.robotMass.in(Kilograms);
  private final double ROBOT_MOI = JsonConstants.robotInfo.robotMOI.in(KilogramSquareMeters);
  private final double WHEEL_COF = JsonConstants.robotInfo.wheelCof;
  private final RobotConfig PP_CONFIG =
      new RobotConfig(
          ROBOT_MASS_KG,
          ROBOT_MOI,
          new ModuleConfig(
              JsonConstants.physicalDriveConstants.FrontLeft.WheelRadius,
              JsonConstants.physicalDriveConstants.kSpeedAt12Volts.in(MetersPerSecond),
              WHEEL_COF,
              DCMotor.getKrakenX60Foc(1)
                  .withReduction(
                      JsonConstants.physicalDriveConstants.FrontLeft.DriveMotorGearRatio),
              JsonConstants.physicalDriveConstants.FrontLeft.SlipCurrent,
              1),
          getModuleTranslations());

  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine sysId;
  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private Rotation2d rawGyroRotation = Rotation2d.kZero;
  private Rotation2d lastGyroYaw = new Rotation2d();
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, Pose2d.kZero);

  private PoseEstimator maPoseEstimator = new PoseEstimator(VecBuilder.fill(0.003, 0.003, 0.0002));

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0, JsonConstants.physicalDriveConstants.FrontLeft);
    modules[1] = new Module(frModuleIO, 1, JsonConstants.physicalDriveConstants.FrontRight);
    modules[2] = new Module(blModuleIO, 2, JsonConstants.physicalDriveConstants.BackLeft);
    modules[3] = new Module(brModuleIO, 3, JsonConstants.physicalDriveConstants.BackRight);

    // Usage reporting for swerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    // Start odometry thread
    PhoenixOdometryThread.getInstance().start();

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configure(
        this::getPose,
        this::setPose,
        this::getChassisSpeeds,
        this::runVelocity,
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
        PP_CONFIG,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[0]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));

    // Since Drive is placed within an Optional, it can't be found in a recursive down from Robot
    AutoLogOutputManager.addObject(this);
  }

  @Override
  public void periodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }
    odometryLock.unlock();

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // The following code was added for the support of ma pose estimator
      Rotation2d deltaYaw;
      if (i == 0) {
        deltaYaw = rawGyroRotation.minus(lastGyroYaw);
      } else {
        deltaYaw = rawGyroRotation.minus(gyroInputs.odometryYawPositions[i - 1]);
      }
      var twist = kinematics.toTwist2d(moduleDeltas);
      twist = new Twist2d(twist.dx, twist.dy, deltaYaw.getRadians());

      Twist2d totalTwist = new Twist2d();
      totalTwist =
          new Twist2d(
              totalTwist.dx + twist.dx, totalTwist.dy + twist.dy, totalTwist.dtheta + twist.dtheta);

      if (gyroInputs.connected) {
        totalTwist.dtheta = gyroInputs.yawPosition.minus(lastGyroYaw).getRadians();
        lastGyroYaw = gyroInputs.yawPosition;
      } else {
        totalTwist.dtheta = rawGyroRotation.minus(lastGyroYaw).getRadians();
        lastGyroYaw = rawGyroRotation;
      }

      // Apply update
      if (JsonConstants.featureFlags.useMAPoseEstimator) {
        maPoseEstimator.addDriveData(sampleTimestamps[i], totalTwist);
      } else {
        poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
      }
    }
    // Update gyro alert
    gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        setpointStates, JsonConstants.physicalDriveConstants.kSpeedAt12Volts);

    // Log unoptimized setpoints and setpoint speeds
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
  }

  public void setGoalSpeedsBlueOrigins(ChassisSpeeds goalSpeeds) {
    Rotation2d robotRotation = getRotation();

    runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(goalSpeeds, robotRotation));
  }

  @Override
  public void setGoalSpeeds(ChassisSpeeds goalSpeeds, boolean isFieldCentric) {
    Logger.recordOutput("drive/goalSpeeds", goalSpeeds);
    if (isFieldCentric) {
      // Adjust for field-centric control
      boolean isFlipped =
          DriverStation.getAlliance().isPresent()
              && DriverStation.getAlliance().get() == Alliance.Red;

      Rotation2d robotRotation =
          isFlipped
              ? getRotation().plus(new Rotation2d(Math.PI)) // Flip orientation for Red Alliance
              : getRotation();

      runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(goalSpeeds, robotRotation));
    } else {
      Logger.recordOutput("Drive/DesiredRobotCentricSpeeds", goalSpeeds);

      runVelocity(goalSpeeds);
    }
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    if (JsonConstants.featureFlags.useMAPoseEstimator) {
      return maPoseEstimator.getLatestPose();
    } else {
      return poseEstimator.getEstimatedPosition();
    }
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    if (JsonConstants.featureFlags.useMAPoseEstimator) {
      maPoseEstimator.resetPose(pose);
    } else {
      poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
    }
  }

  /** Adds a new timestamped vision measurement. */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    if (JsonConstants.featureFlags.useMAPoseEstimator) {
      maPoseEstimator.addVisionData(
          List.of(
              new TimestampedVisionUpdate(
                  timestampSeconds, visionRobotPoseMeters, visionMeasurementStdDevs)));
    } else {
      poseEstimator.addVisionMeasurement(
          visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return JsonConstants.physicalDriveConstants.kSpeedAt12Volts.in(MetersPerSecond);
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return getMaxLinearSpeedMetersPerSec() / JsonConstants.physicalDriveConstants.drive_base_radius;
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(
          JsonConstants.physicalDriveConstants.FrontLeft.LocationX,
          JsonConstants.physicalDriveConstants.FrontLeft.LocationY),
      new Translation2d(
          JsonConstants.physicalDriveConstants.FrontRight.LocationX,
          JsonConstants.physicalDriveConstants.FrontRight.LocationY),
      new Translation2d(
          JsonConstants.physicalDriveConstants.BackLeft.LocationX,
          JsonConstants.physicalDriveConstants.BackLeft.LocationY),
      new Translation2d(
          JsonConstants.physicalDriveConstants.BackRight.LocationX,
          JsonConstants.physicalDriveConstants.BackRight.LocationY)
    };
  }

  public void setSteerGains(PIDGains gains) {
    JsonConstants.driveConstants.steerGains = gains;
    for (var module : modules) {
      module.setSteerGains(gains);
    }
  }

  public void setDriveGains(PIDGains gains) {
    JsonConstants.driveConstants.driveGains = gains;
    for (var module : modules) {
      module.setDriveGains(gains);
    }
  }
}
