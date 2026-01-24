package frc.robot.constants;

import coppercore.parameter_tools.json.annotations.JSONExclude;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterConstants {
  public final Double[] distanceToViDistancesMeters = {1.8, 2.0, 4.0};
  public final Double[] distanceToViVisMetersPerSecond = {6.4, 6.8, 8.8};

  /**
   * The "default initial velocity" that is used if getViFromDistance is called without first
   * initializing the map.
   */
  public final Double defaultViMetersPerSecond = 9.0;

  @JSONExclude private InterpolatingDoubleTreeMap distanceToVi;

  /**
   * Initialize the map of distance to initial velocity
   *
   * <p>This method must be called before getViFromDistance can be used
   */
  public void initializeViMap() {
    if (distanceToViDistancesMeters.length != distanceToViVisMetersPerSecond.length) {
      throw new IllegalArgumentException(
          "distanceToViDistancesMeters and distanceToViVisMetersPerSecond had different lengths in ShooterConstants");
    }

    distanceToVi = new InterpolatingDoubleTreeMap();
    for (int i = 0; i < distanceToViDistancesMeters.length; i++) {
      distanceToVi.put(distanceToViDistancesMeters[i], distanceToViVisMetersPerSecond[i]);
    }
  }

  public double getViFromDistance(double distanceMeters) {
    if (distanceToVi == null) {
      System.err.println(
          "Error: ShooterConsants.getViFromDistance called before map was initialized.");
      return defaultViMetersPerSecond;
    }

    return distanceToVi.get(distanceMeters);
  }
}
