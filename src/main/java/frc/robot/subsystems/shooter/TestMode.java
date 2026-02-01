package frc.robot.subsystems.shooter;

import frc.robot.util.TestModeDescription;

public enum TestMode implements TestModeDescription {
  ShooterClosedLoopTuning("Shooter Closed Loop Tuning"),
  ShooterVoltageTuning("Shooter Voltage Open-loop Tuning"),
  ShooterCurrentTuning("Shooter Current Open-loop Tuning"),
  ShooterPhoenixTuning("Shooter Phoenix (no-op) Tuning"),
  None;

  private final String description;

  TestMode(String description) {
    this.description = description;
  }

  TestMode() {
    this.description = name();
  }

  @Override
  public String getDescription() {
    return this.description;
  }
}
