package frc.robot.util;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import coppercore.wpilib_interface.subsystems.motors.MotorIO;
import coppercore.wpilib_interface.subsystems.motors.profile.MotionProfileConfig;
import coppercore.wpilib_interface.subsystems.motors.profile.MutableMotionProfileConfig;
import frc.robot.util.LoggedTunableMeasure.LoggedAngularAcceleration;
import frc.robot.util.LoggedTunableMeasure.LoggedAngularJerk;
import frc.robot.util.LoggedTunableMeasure.LoggedAngularVelocity;
import frc.robot.util.LoggedTunableMeasure.LoggedVoltagePerAngularAcceleration;
import frc.robot.util.LoggedTunableMeasure.LoggedVoltagePerAngularVelocity;

public class LoggedTunableMotionProfile {
  LoggedAngularVelocity maxVelocity;
  LoggedAngularAcceleration maxAcceleration;
  LoggedAngularJerk maxJerk;
  LoggedVoltagePerAngularVelocity kV;
  LoggedVoltagePerAngularAcceleration kA;

  MutableMotionProfileConfig currentMotionProfile;

  public static final MotionProfileConfig defaultMotionProfileConfig =
      MotionProfileConfig.immutable(
          RotationsPerSecond.zero(),
          RotationsPerSecondPerSecond.zero(),
          RotationsPerSecondPerSecond.zero().div(Seconds.of(1.0)),
          Volts.of(0).div(RotationsPerSecond.of(1)),
          Volts.of(0).div(RotationsPerSecondPerSecond.of(1)));

  public LoggedTunableMotionProfile(String namePrefix, MotionProfileConfig defaultProfile) {
    this.currentMotionProfile = defaultProfile.derive();

    this.maxVelocity =
        LoggedTunableMeasure.ANGULAR_VELOCITY.of(
            namePrefix + "MaxVelocity", defaultProfile.getMaxVelocity(), RotationsPerSecond);
    this.maxAcceleration =
        LoggedTunableMeasure.ANGULAR_ACCELERATION.of(
            namePrefix + "MaxAcceleration", defaultProfile.getMaxAcceleration(), RotationsPerSecondPerSecond);
    this.maxJerk =
        LoggedTunableMeasure.ANGULAR_JERK.of(
            namePrefix + "MaxJerk", defaultProfile.getMaxJerk(), RotationsPerSecondPerSecond.per(Seconds));
    this.kV =
        LoggedTunableMeasure.VOLTAGE_PER_ANGULAR_VELOCITY.of(
            namePrefix + "kV", defaultProfile.getExpoKv(), Volts.per(RotationsPerSecond));
    this.kA =
        LoggedTunableMeasure.VOLTAGE_PER_ANGULAR_ACCELERATION.of(
            namePrefix + "kA", defaultProfile.getExpoKa(), Volts.per(RotationsPerSecondPerSecond));
  }

  public LoggedTunableMotionProfile(String namePrefix) {
    this(namePrefix, defaultMotionProfileConfig);
  }

  public MotionProfileConfig getCurrentMotionProfile() {
    updateProfile(hashCode());
    return currentMotionProfile;
  }

  private boolean updateProfile(int id) {
    boolean hasChanged = false;
    if (maxVelocity.hasChanged()) {
      currentMotionProfile.withMaxVelocity(maxVelocity.get());
      hasChanged = true;
    }
    if (maxAcceleration.hasChanged()) {
      currentMotionProfile.withMaxAcceleration(maxAcceleration.get());
      hasChanged = true;
    }
    if (maxJerk.hasChanged()) {
      currentMotionProfile.withMaxJerk(maxJerk.get());
      hasChanged = true;
    }
    if (kV.hasChanged()) {
      currentMotionProfile.withExpoKv(kV.get());
      hasChanged = true;
    }
    if (kA.hasChanged()) {
      currentMotionProfile.withExpoKa(kA.get());
      hasChanged = true;
    }
    return hasChanged;
  }

  public void ifChanged(int id, MotionProfileConsumer callback) {
    boolean hasChanged = updateProfile(id);
    if (hasChanged) {
      callback.accept(currentMotionProfile);
    }
  }

  public MotionProfileConsumer getMotorIOApplier(MotorIO motorIO) {
    return motionProfile -> motorIO.setProfileConstraints(motionProfile);
  }

  public MotionProfileConsumer getMotorIOAppliers(MotorIO... motorIOs) {
    return motionProfile -> {
      for (MotorIO motorIO : motorIOs) {
        motorIO.setProfileConstraints(motionProfile);
      }
    };
  }

  @FunctionalInterface
  public interface MotionProfileConsumer {
    MotionProfileConsumer noOp = motionProfile -> {};

    void accept(MotionProfileConfig motionProfile);

    default MotionProfileConsumer chain(MotionProfileConsumer after) {
      return motionProfile -> {
        accept(motionProfile);
        after.accept(motionProfile);
      };
    }

    static MotionProfileConsumer noOp() {
      return noOp;
    }
  }
}
