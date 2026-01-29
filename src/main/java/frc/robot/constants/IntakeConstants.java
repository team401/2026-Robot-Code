package frc.robot.constants;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.MomentOfInertia;

public class IntakeConstants {
  public final MomentOfInertia pivotInertia = Units.KilogramSquareMeters.of(0.05);
  public final Double pivotGearing = 1.0 / 100.0;
  public final Double armLengthMeters = 0.3;
  public final Double minPivotAngleRadians = -Math.PI / 2;
  public final Double maxPivotAngleRadians = Math.PI / 2;
  public final Double pivotStartingAngleRadians = 0.0;
  public final MomentOfInertia rollersInertia = Units.KilogramSquareMeters.of(0.02);

  // Setpoints for various positions
  public final Double intakePositionAngleRadians = 0.0;
  public final Double stowPositionAngleRadians = Math.PI / 2;

  // Roller speeds
  public final Double intakeRollerSpeedRPM = 1500.0;
}
