// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.constants.JsonConstants;
import java.util.Queue;

/** IO implementation for Pigeon 2. */
public class GyroIOPigeon2 implements GyroIO {
  // Maximum number of odometry samples we expect per cycle
  private static final int MAX_ODOMETRY_SAMPLES = 20;

  private final Pigeon2 pigeon =
      new Pigeon2(
          JsonConstants.physicalDriveConstants.DrivetrainConstants.Pigeon2Id,
          JsonConstants.robotInfo.CANBus);
  private final StatusSignal<Angle> yaw = pigeon.getYaw();
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;
  private final StatusSignal<AngularVelocity> yawVelocity = pigeon.getAngularVelocityZWorld();

  /*
   * Pre-allocated arrays for odometry to avoid per-cycle allocations from stream operations.
   * [Optimization by Claude Opus 4.5, March 2026]
   */
  private final double[] preallocatedTimestamps = new double[MAX_ODOMETRY_SAMPLES];
  private final Rotation2d[] preallocatedYawPositions = new Rotation2d[MAX_ODOMETRY_SAMPLES];

  public GyroIOPigeon2() {
    if (JsonConstants.physicalDriveConstants.DrivetrainConstants.Pigeon2Configs != null) {
      pigeon
          .getConfigurator()
          .apply(JsonConstants.physicalDriveConstants.DrivetrainConstants.Pigeon2Configs);
    } else {
      pigeon.getConfigurator().apply(new Pigeon2Configuration());
    }

    pigeon.getConfigurator().setYaw(0.0);
    yaw.setUpdateFrequency(JsonConstants.robotInfo.ODOMETRY_FREQUENCY);
    yawVelocity.setUpdateFrequency(50.0);
    pigeon.optimizeBusUtilization();
    yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(yaw.clone());
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);
    inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());

    // ===== OPTIMIZED ODOMETRY UPDATE (NO STREAM ALLOCATIONS) =====
    // Drain timestamp queue into pre-allocated array
    int sampleCount = 0;
    Double timestamp;
    while ((timestamp = yawTimestampQueue.poll()) != null && sampleCount < MAX_ODOMETRY_SAMPLES) {
      preallocatedTimestamps[sampleCount] = timestamp;
      sampleCount++;
    }

    // Drain yaw position queue
    int yawCount = 0;
    Double yawPos;
    while ((yawPos = yawPositionQueue.poll()) != null && yawCount < MAX_ODOMETRY_SAMPLES) {
      preallocatedYawPositions[yawCount] = Rotation2d.fromDegrees(yawPos);
      yawCount++;
    }

    int finalCount = Math.min(sampleCount, yawCount);

    // Only allocate new arrays if size changed
    if (inputs.odometryYawTimestamps.length != finalCount) {
      inputs.odometryYawTimestamps = new double[finalCount];
    }
    if (inputs.odometryYawPositions.length != finalCount) {
      inputs.odometryYawPositions = new Rotation2d[finalCount];
    }

    // Copy from pre-allocated arrays
    System.arraycopy(preallocatedTimestamps, 0, inputs.odometryYawTimestamps, 0, finalCount);
    System.arraycopy(preallocatedYawPositions, 0, inputs.odometryYawPositions, 0, finalCount);
  }
}
