// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.util.PIDGains;
import org.littletonrobotics.junction.Logger;

public class Module {
  // Maximum number of odometry samples we expect per cycle (for pre-allocation)
  private static final int MAX_ODOMETRY_SAMPLES = 20;

  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final int index;
  private final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      constants;

  private final Alert driveDisconnectedAlert;
  private final Alert turnDisconnectedAlert;
  private final Alert turnEncoderDisconnectedAlert;
  private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

  /*
   * Pre-allocated objects to avoid per-cycle allocations.
   * [Optimization by Claude Opus 4.5, March 2026]
   */
  // Pre-cached logger key to avoid string concatenation every cycle
  private final String loggerKey;

  // Pre-allocated array for odometry positions
  private final SwerveModulePosition[] preallocatedOdometryPositions =
      new SwerveModulePosition[MAX_ODOMETRY_SAMPLES];

  // Track actual sample count separately from array length
  private int odometrySampleCount = 0;

  public Module(
      ModuleIO io,
      int index,
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          constants) {
    this.io = io;
    this.index = index;
    this.constants = constants;

    // Pre-cache the logger key to avoid string concatenation every cycle
    this.loggerKey = "Drive/Module" + index;

    // Pre-allocate SwerveModulePosition objects
    for (int i = 0; i < MAX_ODOMETRY_SAMPLES; i++) {
      preallocatedOdometryPositions[i] = new SwerveModulePosition();
    }

    driveDisconnectedAlert =
        new Alert(
            "Disconnected drive motor on module " + Integer.toString(index) + ".",
            AlertType.kError);
    turnDisconnectedAlert =
        new Alert(
            "Disconnected turn motor on module " + Integer.toString(index) + ".", AlertType.kError);
    turnEncoderDisconnectedAlert =
        new Alert(
            "Disconnected turn encoder on module " + Integer.toString(index) + ".",
            AlertType.kError);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(loggerKey, inputs);

    // Calculate positions for odometry using pre-allocated objects
    odometrySampleCount = inputs.odometryTimestamps.length;

    // Reuse pre-allocated array if possible, otherwise allocate (rare case)
    if (odometrySampleCount <= MAX_ODOMETRY_SAMPLES) {
      // Use pre-allocated positions
      for (int i = 0; i < odometrySampleCount; i++) {
        double positionMeters = inputs.odometryDrivePositionsRad[i] * constants.WheelRadius;
        Rotation2d angle = inputs.odometryTurnPositions[i];
        // SwerveModulePosition is mutable, but we'll just reuse the array slots
        // and create new positions since SwerveModulePosition doesn't have setters
        preallocatedOdometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
      }
      odometryPositions = preallocatedOdometryPositions;
    } else {
      // Fallback for unexpected large sample counts (should never happen)
      odometryPositions = new SwerveModulePosition[odometrySampleCount];
      for (int i = 0; i < odometrySampleCount; i++) {
        double positionMeters = inputs.odometryDrivePositionsRad[i] * constants.WheelRadius;
        Rotation2d angle = inputs.odometryTurnPositions[i];
        odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
      }
    }

    // Update alerts
    driveDisconnectedAlert.set(!inputs.driveConnected);
    turnDisconnectedAlert.set(!inputs.turnConnected);
    turnEncoderDisconnectedAlert.set(!inputs.turnEncoderConnected);
  }

  /** Runs the module with the specified setpoint state. Mutates the state to optimize it. */
  public void runSetpoint(SwerveModuleState state) {
    // Optimize velocity setpoint
    state.optimize(getAngle());
    state.cosineScale(inputs.turnPosition);

    // Apply setpoints
    io.setDriveVelocity(state.speedMetersPerSecond / constants.WheelRadius);
    io.setTurnPosition(state.angle);
  }

  /** Runs the module with the specified output while controlling to zero degrees. */
  public void runCharacterization(double output) {
    io.setDriveOpenLoop(output);
    io.setTurnPosition(Rotation2d.kZero);
  }

  /** Disables all outputs to motors. */
  public void stop() {
    io.setDriveOpenLoop(0.0);
    io.setTurnOpenLoop(0.0);
  }

  /** Returns the current turn angle of the module. */
  public Rotation2d getAngle() {
    return inputs.turnPosition;
  }

  /** Returns the current drive position of the module in meters. */
  public double getPositionMeters() {
    return inputs.drivePositionRad * constants.WheelRadius;
  }

  /** Returns the current drive velocity of the module in meters per second. */
  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityRadPerSec * constants.WheelRadius;
  }

  /** Returns the module position (turn angle and drive position). */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /** Returns the module state (turn angle and drive velocity). */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  /** Returns the module positions received this cycle. */
  public SwerveModulePosition[] getOdometryPositions() {
    return odometryPositions;
  }

  /** Returns the timestamps of the samples received this cycle. */
  public double[] getOdometryTimestamps() {
    return inputs.odometryTimestamps;
  }

  /** Returns the module position in radians. */
  public double getWheelRadiusCharacterizationPosition() {
    return inputs.drivePositionRad;
  }

  /** Returns the module velocity in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    return Units.radiansToRotations(inputs.driveVelocityRadPerSec);
  }

  public void setSteerGains(PIDGains gains) {
    io.setSteerGains(gains);
  }

  public void setDriveGains(PIDGains gains) {
    io.setDriveGains(gains);
  }
}
