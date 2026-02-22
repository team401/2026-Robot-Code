package frc.robot.coordination;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.JsonConstants;
import frc.robot.constants.StrategyConstants;
import frc.robot.util.AllianceUtil;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.Logger;

/**
 * The MatchState class handles tracking the state of the match and indicating when we can score vs.
 * not score.
 */
public class MatchState {
  private Optional<Alliance> wonAuto = Optional.empty();
  private boolean canTrustFMS = false;

  private MatchShift currentShift = MatchShift.Unknown;
  private boolean canScore = true;

  /**
   * @return {@code true} if we are in a match (FMS is attached or DriverStation.getMatchType()
   *     return Practice, Qualification, or Elimination), {@code false} otherwise
   */
  @AutoLogOutput(key = "MatchState/isInMatch")
  public boolean isInMatch() {
    return DriverStation.isFMSAttached()
        || DriverStation.getMatchType() == DriverStation.MatchType.Practice
        || DriverStation.getMatchType() == DriverStation.MatchType.Qualification
        || DriverStation.getMatchType() == DriverStation.MatchType.Elimination;
  }

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
    /** Shop/pit testing; allow shooting just in case */
    Testing(ActiveHub.Both);

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
     * Checks whether our alliance's hub is currently active, given information about who won auto
     * and which alliance we are on
     *
     * @param autoWinner The winner of auto (or red if we haven't determined it yet)
     * @param hasDeterminedAutoWinner Whether or not we have determined the auto winner
     * @return {@code true} if, based on this ActiveHub value, our hub should be active, or if the
     *     auto winner is unknown, and {@code false} otherwise.
     */
    public boolean isOurHubActive(Optional<Alliance> autoWinner) {
      Alliance ourAlliance = AllianceUtil.getAlliance();

      switch (this) {
        case Both -> {
          return true;
        }
        case AutoWinner -> {
          return autoWinner.map(winningAlliance -> ourAlliance == winningAlliance).orElse(true);
        }
        case AutoLoser -> {
          return autoWinner.map(winningAlliance -> ourAlliance != winningAlliance).orElse(true);
        }
        default -> {
          throw new UnsupportedOperationException("Unknown ActiveHub value " + this);
        }
      }
    }
  }

  public MatchState() {
    AutoLogOutputManager.addObject(this);

    SmartDashboard.putBoolean("matchState/manualRedOverride", false);
    SmartDashboard.putBoolean("matchState/manualBlueOverride", false);
  }

  /** Must be called by the coordination layer when enabled */
  public void enabledPeriodic(boolean manualRedOverridePressed, boolean manualBlueOverridePressed) {
    checkGameDataForAutoWinner();
    if (!wonAuto.isPresent()) {
      // Check for game data; if not present allow for manual override.
      if (manualRedOverridePressed && !canTrustFMS) {
        wonAuto = Optional.of(Alliance.Red);
      }
      if (manualBlueOverridePressed && !canTrustFMS) {
        wonAuto = Optional.of(Alliance.Blue);
      }
    }

    Logger.recordOutput("MatchState/canTrustFMS", canTrustFMS);
    Logger.recordOutput("MatchState/manualRedOverridePressed", manualRedOverridePressed);
    Logger.recordOutput("MatchState/manualBlueOverridePressed", manualBlueOverridePressed);
    Logger.recordOutput("MatchState/autoWinner", wonAuto.orElse(null));
    // Determine if it's possible to shoot by checking the current match shift
    double matchTime = DriverStation.getMatchTime();
    currentShift = getCurrentShift(matchTime);

    double shiftStartGracePeriodSeconds =
        JsonConstants.strategyConstants.shiftStartGracePeriod.in(Seconds);
    MatchShift shiftWithStartGrace = getCurrentShift(matchTime + shiftStartGracePeriodSeconds);

    Logger.recordOutput("MatchState/startGraceShift", shiftWithStartGrace);

    double shiftEndGracePeriodSeconds =
        JsonConstants.strategyConstants.shiftEndGracePeriod.in(Seconds);
    MatchShift shiftWithEndGrace = getCurrentShift(matchTime - shiftEndGracePeriodSeconds);

    Logger.recordOutput("MatchState/endGraceShift", shiftWithEndGrace);

    this.canScore =
        currentShift.getActiveHub().isOurHubActive(wonAuto)
            || shiftWithStartGrace.getActiveHub().isOurHubActive(wonAuto)
            || shiftWithEndGrace.getActiveHub().isOurHubActive(wonAuto);
  }

  private void checkGameDataForAutoWinner() {
    String gameData = DriverStation.getGameSpecificMessage();
    if (gameData.length() > 0) {
      if (gameData.startsWith("R")) {
        wonAuto = Optional.of(Alliance.Red);
        canTrustFMS = true;
      } else if (gameData.startsWith("B")) {
        wonAuto = Optional.of(Alliance.Blue);
        canTrustFMS = true;
      }
    } else {
      canTrustFMS = false;
    }
  }

  public Optional<Alliance> getAutoWinner() {
    return wonAuto;
  }

  public MatchShift getCurrentShift(double currentMatchTime) {
    // The behavior of this method is determined largely by the docs here
    // https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj/DriverStation.html#getMatchTime()
    if (isInMatch()) {
      // Either FMS is attached or time is counting down. Either way, time is decreasing so we are
      // in a "match" situation
      if (DriverStation.isAutonomous()) {
        return MatchShift.Auto;
      } else {
        return getTeleopShiftFromMatchTime(currentMatchTime);
      }
    }

    return MatchShift.Testing;
    // TODO: Determine if having "Unknown" makes any sense considering that right now, we are forced
    // to either return a real shift or "Testing"
  }

  private MatchShift getTeleopShiftFromMatchTime(double matchTime) {
    // see page 43 https://firstfrc.blob.core.windows.net/frc2026/Manual/2026GameManual.pdf
    if (matchTime <= StrategyConstants.transitionStart
        && matchTime > StrategyConstants.shift1Start) {
      // Transition period; both hubs active
      return MatchShift.Transition;
    } else if (matchTime <= StrategyConstants.shift1Start
        && matchTime > StrategyConstants.shift2Start) {
      // Shift 1; auto loser's hub active
      return MatchShift.Shift1;
    } else if (matchTime <= StrategyConstants.shift2Start
        && matchTime > StrategyConstants.shift3Start) {
      // Shift 2; auto winner's hub active
      return MatchShift.Shift2;
    } else if (matchTime <= StrategyConstants.shift3Start
        && matchTime > StrategyConstants.shift4Start) {
      // Shift 3; auto loser's hub active
      return MatchShift.Shift3;
    } else if (matchTime <= StrategyConstants.shift4Start
        && matchTime > StrategyConstants.endgameStart) {
      // Shift 4; auto winner's hub active
      return MatchShift.Shift4;
    } else if (matchTime <= StrategyConstants.endgameStart
        && matchTime > StrategyConstants.matchEnd) {
      // Endgame; both hubs active
      return MatchShift.Endgame;
    }

    return MatchShift.Unknown;
  }

  public double getTimeLeftInCurrentShift() {
    double matchTime = DriverStation.getMatchTime();
    double timeLeft;

    if (currentShift == MatchShift.Transition) {
      timeLeft = Math.max(matchTime - StrategyConstants.shift1Start, 0);
    } else if (currentShift == MatchShift.Shift1) {
      timeLeft = Math.max(matchTime - StrategyConstants.shift2Start, 0);
    } else if (currentShift == MatchShift.Shift2) {
      timeLeft = Math.max(matchTime - StrategyConstants.shift3Start, 0);
    } else if (currentShift == MatchShift.Shift3) {
      timeLeft = Math.max(matchTime - StrategyConstants.shift4Start, 0);
    } else if (currentShift == MatchShift.Shift4) {
      timeLeft = Math.max(matchTime - StrategyConstants.endgameStart, 0);
    } else if (currentShift == MatchShift.Endgame) {
      timeLeft = Math.max(matchTime - StrategyConstants.matchEnd, 0);
    } else {
      timeLeft = Double.POSITIVE_INFINITY;
    }

    Logger.recordOutput("MatchState/timeLeftInShift", timeLeft);
    Logger.recordOutput(
        "MatchState/alliance", DriverStation.getAlliance().map(Enum::name).orElse("Unknown"));

    return timeLeft;
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

  @AutoLogOutput(key = "MatchState/hasDeterminedAutoWinner")
  public boolean hasDeterminedAutoWinner() {
    return wonAuto.isPresent();
  }
}
