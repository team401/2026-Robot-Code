package frc.robot.constants;

import static edu.wpi.first.units.Units.Seconds;

import coppercore.vision.VisionGainConstants;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Time;

public class VisionConstants {
  // Placeholder values for camera configs
  public final VisionGainConstants gainConstants = new VisionGainConstants();
  public final Integer camera0Index = 0;
  public String camera0Name = "FrontRight";
  public final Transform3d camera0Transform =
      new Transform3d(
          Units.inchesToMeters(1.0),
          Units.inchesToMeters(1.0),
          Units.inchesToMeters(1.0),
          new Rotation3d(0, 0, 0));
  public final Integer camera1Index = 1;
  public String camera1Name = "BackRight";
  public final Transform3d camera1Transform =
      new Transform3d(
          Units.inchesToMeters(2.0),
          Units.inchesToMeters(2.0),
          Units.inchesToMeters(2.0),
          new Rotation3d(0, 0, 0));
  public final Integer camera2Index = 2;
  public String camera2Name = "BackLeft";
  public final Transform3d camera2Transform =
      new Transform3d(
          Units.inchesToMeters(3.0),
          Units.inchesToMeters(3.0),
          Units.inchesToMeters(3.0),
          new Rotation3d(0, 0, 0));

  public final Time disconnectedDebounceTime = Seconds.of(2.0);
}
