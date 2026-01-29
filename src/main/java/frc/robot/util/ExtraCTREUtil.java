package frc.robot.util;

import com.ctre.phoenix6.signals.S1CloseStateValue;
import com.ctre.phoenix6.signals.S1StateValue;
import com.ctre.phoenix6.signals.S2CloseStateValue;
import com.ctre.phoenix6.signals.S2StateValue;

/** Extra methods that belong in Coppercore CTRE util eventually */
public class ExtraCTREUtil {
  private ExtraCTREUtil() {}

  /**
   * The DigitalSignalCloseState enum provides a common type for CTRE's signal-specific close-state
   * values
   */
  public enum DigitalSignalCloseState {
    CloseWhenLow,
    CloseWhenFloating,
    CloseWhenHigh,
    CloseWhenNotLow,
    CloseWhenNotFloating,
    CloseWhenNotHigh;

    public static DigitalSignalCloseState from(S1CloseStateValue s1CloseStateValue) {
      return switch (s1CloseStateValue) {
        case CloseWhenLow -> CloseWhenLow;
        case CloseWhenFloating -> CloseWhenFloating;
        case CloseWhenHigh -> CloseWhenHigh;
        case CloseWhenNotLow -> CloseWhenNotLow;
        case CloseWhenNotFloating -> CloseWhenNotFloating;
        case CloseWhenNotHigh -> CloseWhenNotHigh;
      };
    }

    public static DigitalSignalCloseState from(S2CloseStateValue s2CloseStateValue) {
      return switch (s2CloseStateValue) {
        case CloseWhenLow -> CloseWhenLow;
        case CloseWhenFloating -> CloseWhenFloating;
        case CloseWhenHigh -> CloseWhenHigh;
        case CloseWhenNotLow -> CloseWhenNotLow;
        case CloseWhenNotFloating -> CloseWhenNotFloating;
        case CloseWhenNotHigh -> CloseWhenNotHigh;
      };
    }

    public S1StateValue getS1Closed() {
      return switch (this) {
        case CloseWhenLow -> S1StateValue.Low;
        case CloseWhenFloating -> S1StateValue.Floating;
        case CloseWhenHigh -> S1StateValue.High;
        case CloseWhenNotLow -> S1StateValue.High;
        case CloseWhenNotFloating -> S1StateValue.Low;
        case CloseWhenNotHigh -> S1StateValue.Low;
      };
    }

    public S1StateValue getS1Open() {
      return switch (this) {
        case CloseWhenLow -> S1StateValue.High;
        case CloseWhenFloating -> S1StateValue.Low;
        case CloseWhenHigh -> S1StateValue.Low;
        case CloseWhenNotLow -> S1StateValue.Low;
        case CloseWhenNotFloating -> S1StateValue.Floating;
        case CloseWhenNotHigh -> S1StateValue.High;
      };
    }

    public S1StateValue getS1FromOpenState(boolean isOpen) {
      if (isOpen) {
        return getS1Open();
      } else {
        return getS1Closed();
      }
    }

    public S2StateValue getS2Closed() {
      return switch (this) {
        case CloseWhenLow -> S2StateValue.Low;
        case CloseWhenFloating -> S2StateValue.Floating;
        case CloseWhenHigh -> S2StateValue.High;
        case CloseWhenNotLow -> S2StateValue.High;
        case CloseWhenNotFloating -> S2StateValue.Low;
        case CloseWhenNotHigh -> S2StateValue.Low;
      };
    }

    public S2StateValue getS2Open() {
      return switch (this) {
        case CloseWhenLow -> S2StateValue.High;
        case CloseWhenFloating -> S2StateValue.Low;
        case CloseWhenHigh -> S2StateValue.Low;
        case CloseWhenNotLow -> S2StateValue.Low;
        case CloseWhenNotFloating -> S2StateValue.Floating;
        case CloseWhenNotHigh -> S2StateValue.High;
      };
    }

    public S2StateValue getS2FromOpenState(boolean isOpen) {
      if (isOpen) {
        return getS2Open();
      } else {
        return getS2Closed();
      }
    }
  }
}
