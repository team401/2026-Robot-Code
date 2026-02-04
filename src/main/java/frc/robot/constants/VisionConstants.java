package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
  // Placeholder values for camera configs
  public final Integer camera1Index = 0;
  public final Transform3d camera1Transform =
      new Transform3d(
          Units.inchesToMeters(1.0),
          Units.inchesToMeters(1.0),
          Units.inchesToMeters(1.0),
          new Rotation3d(0, 0, 0));
  public final Integer camera2Index = 1;
  public final Transform3d camera2Transform =
      new Transform3d(
          Units.inchesToMeters(2.0),
          Units.inchesToMeters(2.0),
          Units.inchesToMeters(2.0),
          new Rotation3d(0, 0, 0));
  public final Integer camera3Index = 2;
  public final Transform3d camera3Transform =
      new Transform3d(
          Units.inchesToMeters(3.0),
          Units.inchesToMeters(3.0),
          Units.inchesToMeters(3.0),
          new Rotation3d(0, 0, 0));
}
