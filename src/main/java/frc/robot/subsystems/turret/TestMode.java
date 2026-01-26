package frc.robot.subsystems.turret;

import frc.robot.util.TestModeDescription;

public enum TestMode implements TestModeDescription {
  // Turret-related test modes
  TurretVoltageTuning("Turret Voltage Open Loop Tuning"),
  TurretCurrentTuning("Turret Current Open Loop Tuning"),
  TurretClosedLoopTuning("Turret Closed Loop Tuning"),
  TurretPhoenixTuning("Turret Phoenix Tuning (no-op)"),
  // Hopper-related test modes
  HopperVoltageTuning("Hopper Voltage Open Loop Tuning"),
  HopperCurrentTuning("Hopper Current Open Loop Tuning"),
  HopperClosedLoopTuning("Hopper Closed Loop Tuning"),
  HopperPhoenixTuning("Hopper Phoenix Tuning (no-op)"),
  None; // No test mode selected

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
