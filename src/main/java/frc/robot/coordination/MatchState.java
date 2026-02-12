package frc.robot.coordination;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.JsonConstants;
import frc.robot.util.AllianceUtil;
import java.util.function.DoublePredicate;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.Logger;

/**
 * The MatchState class handles tracking the state of the match and indicating when we can score vs.
 * not score.
 */
public class MatchState {
  private Alliance wonAuto = Alliance.Red;
  private boolean hasDeterminedAutoWinner = false;

  /** Track the last match time to determine when we are using practice mode */
  private double lastMatchTime = 0.0;

  private MatchShift currentShift = MatchShift.Unknown;
  private boolean canScore = true;

  public enum MatchShift {
    /** Autonomous; both hubs are active */
    Auto(ActiveHub.Both),
    /** Transition shift; both hubs are active */
    Transition(ActiveHub.Both),
    /** Shift 1; auto loser's hub is active */
    Shift1(ActiveHub.AutoLoser),
    /** Shift 2; auto winner's hub is active */
    Shift2(ActiveHub.AutoWinner),
    /** Shift 3; auto loser's hub is active */
    Shift3(ActiveHub.AutoLoser),
    /** Shift 4; auto winner's hub is active */
    Shift4(ActiveHub.AutoWinner),
    /** Endgame; both hubs are active */
    Endgame(ActiveHub.Both),
    /** Unknown; allow shooting just in case */
    Unknown(ActiveHub.Both),
    /** Shop testing; allow shooting just in case */
    AtHome(ActiveHub.Both);

    private ActiveHub hub;

    private MatchShift(ActiveHub hub) {
      this.hub = hub;
    }

    public ActiveHub getActiveHub() {
      return hub;
    }
  }

  public enum ActiveHub {
    Both,
    AutoWinner,
    AutoLoser;

    /**
     * Checks whether the hub is currently active, given information about who won auto and which
     * alliance we are on
     *
     * @param autoWinner The winner of auto (or red if we haven't determined it yet)
     * @param hasDeterminedAutoWinner Whether or not we have determined the auto winner
     * @return
     */
    public boolean isHubActive(Alliance autoWinner, boolean hasDeterminedAutoWinner) {
      Alliance ourAlliance = AllianceUtil.getAlliance();

      if (this == Both) {
        return true;
      } else if (!hasDeterminedAutoWinner) {
        return true;
      } else if (this == AutoWinner && ourAlliance == autoWinner) {
        return true;
      } else if (this == AutoLoser && ourAlliance != autoWinner) {
        return true;
      } else {
        return false;
      }
    }
  }

  public MatchState() {
    AutoLogOutputManager.addObject(this);
    SmartDashboard.putData(
        "matchState/resetAutoWinner", new InstantCommand(() -> hasDeterminedAutoWinner = false));
  }

  /** Must be called by the coordination layer when enabled */
  public void enabledPeriodic(boolean manualRedOverridePressed, boolean manualBlueOverridePressed) {
    if (!hasDeterminedAutoWinner) {
      checkGameDataForAutoWinner();
    }

    // Determine if it's possible to shoot by checking the current match shift
    double matchTime = DriverStation.getMatchTime();
    currentShift = getCurrentShift(matchTime, lastMatchTime);

    double shiftStartGracePeriodSeconds =
        JsonConstants.strategyConstants.shiftStartGracePeriod.in(Seconds);
    MatchShift shiftWithStartGrace =
        getCurrentShift(
            matchTime + shiftStartGracePeriodSeconds, lastMatchTime + shiftStartGracePeriodSeconds);

    Logger.recordOutput("MatchState/startGraceShift", shiftWithStartGrace);

    double shiftEndGracePeriodSeconds =
        JsonConstants.strategyConstants.shiftEndGracePeriod.in(Seconds);
    MatchShift shiftWithEndGrace =
        getCurrentShift(
            matchTime - shiftEndGracePeriodSeconds, lastMatchTime - shiftEndGracePeriodSeconds);

    Logger.recordOutput("MatchState/endGraceShift", shiftWithEndGrace);

    lastMatchTime = matchTime;

    this.canScore =
        currentShift.getActiveHub().isHubActive(wonAuto, hasDeterminedAutoWinner)
            || shiftWithStartGrace.getActiveHub().isHubActive(wonAuto, hasDeterminedAutoWinner)
            || shiftWithEndGrace.getActiveHub().isHubActive(wonAuto, hasDeterminedAutoWinner);
  }

  private void checkGameDataForAutoWinner() {
    String gameData = DriverStation.getGameSpecificMessage();
    if (gameData.length() > 0) {
      if (gameData.startsWith("R")) {
        wonAuto = Alliance.Red;
        hasDeterminedAutoWinner = true;
      } else if (gameData.startsWith("B")) {
        wonAuto = Alliance.Blue;
        hasDeterminedAutoWinner = true;
      }
    }
  }

  public boolean hasDeterminedAutoWinner() {
    return hasDeterminedAutoWinner;
  }

  public Alliance getAutoWinner() {
    return wonAuto;
  }

  public MatchShift getCurrentShift(double currentMatchTime, double lastMatchTime) {
    // The behavior of this method is determined largely by the docs here
    // https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj/DriverStation.html#getMatchTime()
    if (DriverStation.isFMSAttached() || currentMatchTime <= lastMatchTime) {
      Logger.recordOutput("MatchState/isInMatch", true);
      // Either FMS is attached or time is counting down. Either way, time is decreasing so we are
      // in a "match" situation
      if (DriverStation.isAutonomous()) {
        return MatchShift.Auto;
      } else {
        return getTeleopShiftFromMatchTime(currentMatchTime);
      }
    }
    Logger.recordOutput("MatchState/isInMatch", false);

    if (currentMatchTime > lastMatchTime) {
      // If match time is counting up, we are not in practice mode and therefore we should be
      // allowed to shoot always
      return MatchShift.AtHome;
    }

    return MatchShift.Unknown;
  }

  private MatchShift getTeleopShiftFromMatchTime(double matchTime) {
    // see page 43 https://firstfrc.blob.core.windows.net/frc2026/Manual/2026GameManual.pdf
    if (matchTime <= (2 * 60 + 20) && matchTime > (2 * 60 + 10)) {
      // Transition shift, 2:20 - 2:10
      return MatchShift.Transition;
    } else if (matchTime <= (2 * 60 + 10) && matchTime > (60 + 45)) {
      // Shift 1, 2:10 - 1:45
      return MatchShift.Shift1;
    } else if (matchTime <= (60 + 45) && matchTime > (60 + 20)) {
      // Shift 2, 1:45 - 1:20
      return MatchShift.Shift2;
    } else if (matchTime <= (60 + 20) && matchTime > 55) {
      // Shift 3, 1:20 - 0:55
      return MatchShift.Shift3;
    } else if (matchTime <= 55 && matchTime > 30) {
      // Shift 4, 0:55 - 0:30
      return MatchShift.Shift4;
    } else if (matchTime <= 30 && matchTime > 0) {
      // Endgame, 0:30 - 0:00
      return MatchShift.Endgame;
    }

    return MatchShift.Unknown;
  }

  public DoublePredicate getTimeLeftInCurrentShift() {
    double matchTime = DriverStation.getMatchTime();
    double timeLeft;

    if (currentShift == MatchShift.Transition) {
      // 2:10 remaining -> 130 seconds
      timeLeft = Math.max(matchTime - 130, 0);
    } else if (currentShift == MatchShift.Shift1) {
      // 1:45 remaining -> 105 seconds
      timeLeft = Math.max(matchTime - 105, 0);
    } else if (currentShift == MatchShift.Shift2) {
      timeLeft = Math.max(matchTime - 80, 0);
    } else if (currentShift == MatchShift.Shift3) {
      timeLeft = Math.max(matchTime - 55, 0);
    } else if (currentShift == MatchShift.Shift4) {
      timeLeft = Math.max(matchTime - 30, 0);
    } else if (currentShift == MatchShift.Endgame) {
      timeLeft = Math.max(matchTime - 0, 0);
    } else {

      timeLeft = Double.POSITIVE_INFINITY;
    }

    Logger.recordOutput("MatchState/timeLeftInShift", timeLeft);
    Logger.recordOutput(
        "MatchState/alliance", DriverStation.getAlliance().map(Enum::name).orElse("Unknown"));

    // Return a predicate that checks if a requested amount of time is available in this shift
    return (requiredSeconds) -> requiredSeconds <= timeLeft;
  }

  /**
   * Checks whether the current phase of the match allows this alliance to be able to score.
   *
   * @return {@code true} if the hub is currently active, will soon be active, or has just become
   *     inactive, {@code false} otherwise
   */
  @AutoLogOutput(key = "MatchState/canScore")
  public boolean canScore() {
    return canScore;
  }

  @AutoLogOutput(key = "MatchState/currentShift")
  public MatchShift getCurrentShift() {
    return currentShift;
  }
}
