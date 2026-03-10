// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import coppercore.wpilib_interface.subsystems.StatusSignalRefresher;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import frc.robot.constants.JsonConstants;
import java.util.Queue;

/** IO implementation for Pigeon 2. */
public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 pigeon =
      new Pigeon2(
          JsonConstants.physicalDriveConstants.DrivetrainConstants.Pigeon2Id,
          JsonConstants.robotInfo.CANBus);
  private final StatusSignal<Angle> yaw = pigeon.getYaw();
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;
  private final StatusSignal<AngularVelocity> yawVelocity = pigeon.getAngularVelocityZWorld();

  private final StatusSignal<Angle> pitch = pigeon.getPitch();
  private final StatusSignal<Angle> roll = pigeon.getRoll();
  private final StatusSignal<LinearAcceleration> zAccel = pigeon.getAccelerationZ();

  private final BaseStatusSignal[] signals =
      new BaseStatusSignal[] {yaw, yawVelocity, pitch, roll, zAccel};

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

    pitch.setUpdateFrequency(50.0);
    roll.setUpdateFrequency(50.0);
    zAccel.setUpdateFrequency(50.0);

    StatusSignalRefresher.addSignals(JsonConstants.robotInfo.CANBus, signals);
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = BaseStatusSignal.isAllGood(signals);
    inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());

    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(value))
            .toArray(Rotation2d[]::new);
    yawTimestampQueue.clear();
    yawPositionQueue.clear();

    inputs.pitchRadians = pitch.getValue().in(Radians);
    inputs.rollRadians = roll.getValue().in(Radians);
    inputs.zAccelerationMetersPerSecondPerSecond = zAccel.getValue().in(MetersPerSecondPerSecond);
  }
}
