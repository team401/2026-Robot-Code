package frc.robot.constants;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;

/** ManualModeConstants contains constants that define how to shoot when vision isn't used */
public class ManualModeConstants {
  public final Distance assumedPassDistance = Meters.of(4.0);

  public final Rotation2d bluePassHeading = Rotation2d.kZero;
  public final Rotation2d redPassHeading = Rotation2d.k180deg;

  public final Distance assumedHubDistance = Meters.of(3.1);

  public final Rotation2d blueHubHeading = Rotation2d.k180deg;
  public final Rotation2d redHubHeading = Rotation2d.kZero;
}
