package frc.robot.subsystems.turret.states;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import coppercore.controls.state_machine.StateMachine;
import edu.wpi.first.units.AngularVelocityUnit;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.turret.BaseTurretState;
import frc.robot.subsystems.turret.TurretSubsystem;

public class HomingWaitForMovementState extends BaseTurretState {
  @Override
  public void periodic(StateMachine<TurretSubsystem> stateMachine, TurretSubsystem turret) {
    homeTurret(turret);

    final AngularVelocityUnit velocityComparisonUnit = RadiansPerSecond;
    System.out.println("WFMS: " + turret.getTurretVelocity().abs(DegreesPerSecond));
    if (turret.getTurretVelocity().abs(velocityComparisonUnit)
        >= JsonConstants.turretConstants.homingMovementThreshold.in(velocityComparisonUnit)) {
      finish();
    }
  }
}
