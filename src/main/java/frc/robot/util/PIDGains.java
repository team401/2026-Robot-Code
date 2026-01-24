package frc.robot.util;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;

public record PIDGains(double kP, double kI, double kD, double kS, double kV, double kA, double kG) {
  
  public PIDGains(double kP, double kI, double kD) {
    this(kP, kI, kD, 0.0, 0.0, 0.0, 0.0);
  }

  public PIDGains(double kP, double kI, double kD, double kS, double kV, double kA) {
    this(kP, kI, kD, kS, kV, kA, 0.0);
  }

  public double[] asArrayWithoutFeedForward() {
    return new double[] {kP, kI, kD};
  }

  public double[] asArrayWithoutGravity() {
    return new double[] {kP, kI, kD, kS, kV, kA};
  }

  public double[] asArray() {
    return new double[] {kP, kI, kD, kS, kV, kA, kG};
  }

  public void applyToSlot0Config(Slot0Configs slot0Configs) {
    slot0Configs
        .withKP(kP)
        .withKI(kI)
        .withKD(kD)
        .withKS(kS)
        .withKV(kV)
        .withKA(kA)
        .withKG(kG);
  }

  public void applyToSlot1Config(Slot1Configs slot1Configs) {
    slot1Configs
        .withKP(kP)
        .withKI(kI)
        .withKD(kD)
        .withKS(kS)
        .withKV(kV)
        .withKA(kA)
        .withKG(kG);
  }

  public void applyToSlot2Config(Slot2Configs slot2Configs) {
    slot2Configs
        .withKP(kP)
        .withKI(kI)
        .withKD(kD)
        .withKS(kS)
        .withKV(kV)
        .withKA(kA)
        .withKG(kG);
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
}
