package frc.robot.constants;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;

public class StrategyConstants {
  // Due to the presence of these constants, it is necessary for the driver(s) to play an active
  // role in deciding when to score.

  /**
   * How much grace period to "add" to the start of each shooting shift.
   *
   * <p>For example, a value of 5 seconds means that the robot will enable shooting features 5
   * seconds before it estimates that an active shift begins
   *
   * <p>This constant is necessary because getMatchTime does not return an exact match result, and
   * so the match time is simply an estimate.
   */
  public final Time shiftStartGracePeriod = Seconds.of(2.0);

  /**
   * How much grace period to "add" to the end of each shooting shift.
   *
   * <p>For example, a value of 5 seconds means that the robot will only disable shooting features 5
   * seconds after it estimates that an active shift has ended
   *
   * <p>This constant is necessary because getMatchTime does not return an exact match result, and
   * so the match time is simply an estimate.
   */
  public final Time shiftEndGracePeriod = Seconds.of(2.0);

  // list of shift start times in seconds from the start of the match
  // 2:20
  public static final Double transitionStart = 2 * 60 + 20.0; // 140

  // 2:10
  public static final Double shift1Start = 2 * 60 + 10.0; // 130

  // 1:45
  public static final Double shift2Start = 60 + 45.0; // 105

  // 1:20
  public static final Double shift3Start = 60 + 20.0; // 80

  // 0:55
  public static final Double shift4Start = 55.0;

  // 0:30
  public static final Double endgameStart = 30.0;

  // 0:00
  public static final Double matchEnd = 0.0;
}
