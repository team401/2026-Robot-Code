package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
//imma come back to this. i need to get a working led for testing tn
public class GameDataUtil {
  private boolean isRedShift;
  private double matchTime = DriverStation.getMatchTime();

  public enum currentShift {
    NONE,
    TRANSITION_SHIFT,
    SHIFT_ONE,
    SHIFT_TWO,
    SHIFT_THREE,
    SHIFT_FOUR,
    END_GAME
  }

  public void getCurrentActiveShift() {

    String gameData = DriverStation.getGameSpecificMessage();
    if (gameData != null && gameData.length() > 0) {
      if (gameData.charAt(0) == 'B') {
        isRedShift = false;
      } else if (gameData.charAt(0) == 'R') {
        isRedShift = true;
      }
    }
  }

  public void currentShift() {
    currentShift currentShift = GameDataUtil.currentShift.NONE;

    matchTime = DriverStation.getMatchTime();
    if (matchTime <= 140 && matchTime >= 130) {
      currentShift = GameDataUtil.currentShift.TRANSITION_SHIFT;
    } else if (matchTime < 130 && matchTime >= 100) {
      currentShift = GameDataUtil.currentShift.SHIFT_ONE;
    } else if (matchTime < 100 && matchTime >= 70) {
      currentShift = GameDataUtil.currentShift.SHIFT_TWO;
    } else if (matchTime < 70 && matchTime >= 40) {
      currentShift = GameDataUtil.currentShift.SHIFT_THREE;
    } else if (matchTime < 40 && matchTime >= 20) {
      currentShift = GameDataUtil.currentShift.SHIFT_FOUR;
    } else if (matchTime < 20 && matchTime >= 0) {
      currentShift = GameDataUtil.currentShift.END_GAME;
    }
  }

  // public void timeLeftUntilNextShift() {
  //   getCurrentActiveShift();
  //   if (isRedShift = true && (matchTime == 20 || 40 || 60)) {}
  // }
}
