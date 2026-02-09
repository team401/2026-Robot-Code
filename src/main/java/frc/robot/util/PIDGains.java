package frc.robot.util;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import coppercore.wpilib_interface.subsystems.motors.MotorIO;

public record PIDGains(
    double kP, double kI, double kD, double kS, double kG, double kV, double kA) {

  public static PIDGains kPID(double kP, double kI, double kD) {
    return new PIDGains(kP, kI, kD, 0.0, 0.0, 0.0, 0.0);
  }

  public static PIDGains kPIDSGVA(
      double kP, double kI, double kD, double kS, double kG, double kV, double kA) {
    return new PIDGains(kP, kI, kD, kS, kG, kV, kA);
  }

  public static PIDGains kPIDSVA(
      double kP, double kI, double kD, double kS, double kV, double kA) {
    return new PIDGains(kP, kI, kD, kS, 0.0, kV, kA);
  }


  public double[] asArrayWithoutFeedForward() {
    return new double[] {kP, kI, kD};
  }

  public double[] asArray() {
    return new double[] {kP, kI, kD, kS, kG, kV, kA};
  }

  public Slot0Configs applyToSlot0Config(Slot0Configs slot0Configs) {
    slot0Configs.withKP(kP).withKI(kI).withKD(kD).withKS(kS).withKV(kV).withKA(kA).withKG(kG);
    return slot0Configs;
  }

  public Slot1Configs applyToSlot1Config(Slot1Configs slot1Configs) {
    slot1Configs.withKP(kP).withKI(kI).withKD(kD).withKS(kS).withKV(kV).withKA(kA).withKG(kG);
    return slot1Configs;
  }

  public Slot2Configs applyToSlot2Config(Slot2Configs slot2Configs) {
    slot2Configs.withKP(kP).withKI(kI).withKD(kD).withKS(kS).withKG(kG).withKV(kV).withKA(kA);
    return slot2Configs;
  }

  public Slot0Configs toSlot0Config() {
    Slot0Configs slot0Configs = new Slot0Configs();
    applyToSlot0Config(slot0Configs);
    return slot0Configs;
  }

  public Slot1Configs toSlot1Config() {
    Slot1Configs slot1Configs = new Slot1Configs();
    applyToSlot1Config(slot1Configs);
    return slot1Configs;
  }

  public Slot2Configs toSlot2Config() {
    Slot2Configs slot2Configs = new Slot2Configs();
    applyToSlot2Config(slot2Configs);
    return slot2Configs;
  }

  public void applyToMotorIO(MotorIO motorIO) {
    motorIO.setGains(kP, kI, kD, kS, kG, kV, kA);
  }
}
