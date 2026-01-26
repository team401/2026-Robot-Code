package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class RobotInfo {
  public final String canivoreBusName = "canivore";

  // Pose taken from KrayonCAD
  public final Transform3d robotToShooter =
      new Transform3d(
          Units.inchesToMeters(4.175),
          Units.inchesToMeters(-2.088),
          Units.inchesToMeters(17.0),
          new Rotation3d());
}
