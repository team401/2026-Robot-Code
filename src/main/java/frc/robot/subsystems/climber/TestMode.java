package frc.robot.subsystems.climber;

import coppercore.wpilib_interface.tuning.TestModeDescription;

public enum TestMode implements TestModeDescription {
  // Climber-related test modes
  ClimberVoltageTuning("Climber Voltage Open Loop Tuning"),
  ClimberCurrentTuning("Climber Current Open Loop Tuning"),
  ClimberClosedLoopTuning("Climber Closed Loop Tuning"),
  ClimberPhoenixTuning("Climber Phoenix Tuning (no-op)"),
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
