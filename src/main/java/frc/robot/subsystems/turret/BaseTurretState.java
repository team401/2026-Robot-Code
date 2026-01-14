package frc.robot.subsystems.turret;

import coppercore.controls.state_machine.State;

/**
 * The * BaseTurretState abstract class provides a protected interface between turret states and the
 * TurretSubsystem
 */
public abstract class BaseTurretState extends State<TurretSubsystem> {
  protected void homeTurret(TurretSubsystem turret) {
    turret.applyHomingVoltage();
  }

  protected void setTurretPositionToHomedPosition(TurretSubsystem turret) {
    turret.setPositionToHomedPosition();
  }

  protected void coastTurret(TurretSubsystem turret) {
    turret.coast();
  }
}
