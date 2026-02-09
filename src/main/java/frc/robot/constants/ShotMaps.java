package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import coppercore.parameter_tools.LoggedTunableNumber;
import coppercore.parameter_tools.json.annotations.AfterJsonLoad;
import coppercore.parameter_tools.json.annotations.JSONExclude;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import java.util.Arrays;
import java.util.Comparator;
import java.util.function.Function;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * The ShotMaps class contains constants for our "shooter map", which maps disatnce to shooter RPM,
 * hood angle, and, shot time.
 */
public class ShotMaps {
  // Records
  public record ShotMapDataPoint(
      Distance distance, double shooterRPM, Angle hoodAngle, Time flightTime) {}

  /**
   * A ShotMap contains interpolating double treemaps for RPM by distance, hood angle by distance,
   * and flight time by distance.
   *
   * <p>A ShotMap will be provided for shooting at the hub and for passing (where target height is
   * zero).
   *
   * @param rpmByDistanceMeters Maps a distance in meters to a shooter velocity in RPM
   * @param hoodAngleRadiansByDistanceMeters Maps a distance in meters to a hood angle in radians
   * @param flightTimeSecondsByDistanceMeters Maps a distance in meters to a flight time in seconds
   */
  public record ShotMap(
      InterpolatingDoubleTreeMap rpmByDistanceMeters,
      InterpolatingDoubleTreeMap hoodAngleRadiansByDistanceMeters,
      InterpolatingDoubleTreeMap flightTimeSecondsByDistanceMeters) {
    /**
     * Create a ShotMap with all empty shot maps
     *
     * @return A new ShotMap with all maps initialized to empty InterpolatingDoubleTreeMaps
     */
    private static ShotMap empty() {
      return new ShotMap(
          new InterpolatingDoubleTreeMap(),
          new InterpolatingDoubleTreeMap(),
          new InterpolatingDoubleTreeMap());
    }

    private void initializeFromDataPoints(ShotMapDataPoint[] dataPoints) {
      rpmByDistanceMeters.clear();

      hoodAngleRadiansByDistanceMeters.clear();
      flightTimeSecondsByDistanceMeters.clear();
      for (var dataPoint : dataPoints) {
        double distanceMeters = dataPoint.distance.in(Meters);
        rpmByDistanceMeters.put(distanceMeters, dataPoint.shooterRPM());
        hoodAngleRadiansByDistanceMeters.put(distanceMeters, dataPoint.hoodAngle().in(Radians));
        flightTimeSecondsByDistanceMeters.put(distanceMeters, dataPoint.flightTime().in(Seconds));
      }
    }
  }

  // JSON Synced Fields

  /** A list of ShotMapDataPoints for the hub shooter map */
  public ShotMapDataPoint[] hubDataPoints =
      new ShotMapDataPoint[] {
        // Placeholder datapoint to test
        new ShotMapDataPoint(Meters.of(2.0), 1900, Degrees.of(15.0), Seconds.of(2.0))
      };

  /** A list of ShotMapDataPoints for the passing shooter map */
  public ShotMapDataPoint[] passDataPoints =
      new ShotMapDataPoint[] {
        // Placeholder datapoint to test
        new ShotMapDataPoint(Meters.of(4.0), 2500, Degrees.of(40.0), Seconds.of(3.0))
      };

  /**
   * The "extra" delay to add to shot time to compensate for mechanisms reaching their target
   * position.
   */
  public Time mechanismCompensationDelay = Seconds.of(0.1);

  // Initialized fields (instantiated after json load based on loaded constants)
  /** Initializes the maps and then publishes tuning values for adding values to the map */
  @AfterJsonLoad
  public void afterJsonLoad() {
    System.err.println("ShotMaps::afterJsonLoad called");
    initializeMaps();
    publishTuningValues();
  }

  /**
   * Initializes the shot maps based on the dataPoints from JSON. This method will run after JSON is
   * loaded.
   *
   * <p>It can also be run after a test mode modifies the shot data points to recreate the maps
   */
  public void initializeMaps() {
    hubMap.initializeFromDataPoints(hubDataPoints);
    passingMap.initializeFromDataPoints(passDataPoints);

    Function<ShotMapDataPoint, Double> dataPointToDistanceMeters =
        dataPoint -> dataPoint.distance().in(Meters);
    minHubDistanceMeters =
        Arrays.stream(hubDataPoints)
            .map(dataPointToDistanceMeters)
            .min(Comparator.naturalOrder())
            .orElse(Double.MAX_VALUE);
    maxHubDistanceMeters =
        Arrays.stream(hubDataPoints)
            .map(dataPointToDistanceMeters)
            .max(Comparator.naturalOrder())
            .orElse(Double.MIN_VALUE);

    Logger.recordOutput("ShotMaps/minHubDistanceMeters", minHubDistanceMeters);
    Logger.recordOutput("ShotMaps/maxHubDistanceMeters", maxHubDistanceMeters);

    minPassDistanceMeters =
        Arrays.stream(passDataPoints)
            .map(dataPointToDistanceMeters)
            .min(Comparator.naturalOrder())
            .orElse(Double.MAX_VALUE);
    maxPassDistanceMeters =
        Arrays.stream(passDataPoints)
            .map(dataPointToDistanceMeters)
            .max(Comparator.naturalOrder())
            .orElse(Double.MIN_VALUE);

    Logger.recordOutput("ShotMaps/minPassDistanceMeters", minPassDistanceMeters);
    Logger.recordOutput("ShotMaps/maxPassDistanceMeters", maxPassDistanceMeters);
  }

  /**
   * Create a new array with a length increased by 1, copy all contents from the given array, add
   * the new data point, sort the new array, and return the new array.
   *
   * <p>This method exists to easily update the shot maps during tuning.
   *
   * <p>While sorting isn't technically necessary for generating the InterpolatingDoubleTreeMaps, it
   * does make the JSON file produced much easier to use, as all points will be ordered by distance.
   *
   * @param newDataPoint The new datapoint to add to the array
   * @param dataPoints The array of old datapoints
   * @return An array with all datapoints from dataPoints plus newDataPoint, sorted by distance
   */
  private ShotMapDataPoint[] addDataPointToArray(
      ShotMapDataPoint newDataPoint, ShotMapDataPoint[] dataPoints) {
    int n = dataPoints.length;
    ShotMapDataPoint[] newDataPoints = new ShotMapDataPoint[n];
    System.arraycopy(hubDataPoints, 0, newDataPoints, 0, n);
    newDataPoints[n] = newDataPoint;
    Arrays.sort(newDataPoints, Comparator.comparing(dataPoint -> dataPoint.distance().in(Meters)));
    return newDataPoints;
  }

  public void publishTuningValues() {
    LoggedTunableNumber distanceMeters =
        new LoggedTunableNumber("ShotMapTuning/distanceMeters", 0.0);
    LoggedTunableNumber shooterRPM = new LoggedTunableNumber("ShotMapTuning/shooterRPM", 0.0);
    LoggedTunableNumber hoodAngleDegrees =
        new LoggedTunableNumber("ShotMapTuning/hoodAngleDegrees", 0.0);
    LoggedTunableNumber flightTimeSeconds =
        new LoggedTunableNumber("ShotMapTuning/flightTimeSeconds", 0.0);
    LoggedTunableNumber mechanismCompensationTimeSeconds =
        new LoggedTunableNumber(
            "ShotMapTuning/compensationTimeSeconds", mechanismCompensationDelay.in(Seconds));

    Supplier<ShotMapDataPoint> tunedDataPoint =
        () ->
            new ShotMapDataPoint(
                Meters.of(distanceMeters.getAsDouble()),
                shooterRPM.getAsDouble(),
                Degrees.of(hoodAngleDegrees.getAsDouble()),
                Seconds.of(flightTimeSeconds.getAsDouble()));

    Command addPointToHubMapCommand =
        new InstantCommand(
                () -> {
                  hubDataPoints = addDataPointToArray(tunedDataPoint.get(), hubDataPoints);
                  initializeMaps();
                })
            .ignoringDisable(true);

    Command addPointToPassingMapCommand =
        new InstantCommand(
                () -> {
                  passDataPoints = addDataPointToArray(tunedDataPoint.get(), passDataPoints);
                  initializeMaps();
                })
            .ignoringDisable(true);

    Command updateMechanismDelayCommand =
        new InstantCommand(
                () -> {
                  mechanismCompensationDelay =
                      Seconds.of(mechanismCompensationTimeSeconds.getAsDouble());
                })
            .ignoringDisable(true);

    SmartDashboard.putData("ShotMapTuning/addHubDataPoint", addPointToHubMapCommand);
    SmartDashboard.putData("ShotMapTuning/addPassDataPoint", addPointToPassingMapCommand);
    SmartDashboard.putData("ShotMapTuning/updateMechanismDelay", updateMechanismDelayCommand);
  }

  /** ShotMap for shooting at the hub */
  @JSONExclude public final ShotMap hubMap = ShotMap.empty();

  /** ShotMap for passing to the alliance zone */
  @JSONExclude public final ShotMap passingMap = ShotMap.empty();

  /** The minimum distance when shooting at the hub. Updated automatically by initializeMaps() */
  @JSONExclude public double minHubDistanceMeters;

  /** The maximum distance when shooting at the hub. Updated automatically by initializeMaps() */
  @JSONExclude public double maxHubDistanceMeters;

  /** The minimum distance when passing. Updated automatically by initializeMaps() */
  @JSONExclude public double minPassDistanceMeters;

  /** The maximum distance when passing. Updated automatically by initializeMaps() */
  @JSONExclude public double maxPassDistanceMeters;
}
