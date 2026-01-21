package frc.robot.util;

import coppercore.parameter_tools.LoggedTunableNumber;

public class LoggedTunablePIDGains {
  LoggedTunableNumber kP;
  LoggedTunableNumber kI;
  LoggedTunableNumber kD;
  LoggedTunableNumber kS;
  LoggedTunableNumber kV;
  LoggedTunableNumber kA;

  public LoggedTunablePIDGains(String namePrefix) {
    this(namePrefix, new double[] {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  }

  public LoggedTunablePIDGains(String namePrefix, double[] defaultValues) {
    if (defaultValues.length != 6) {
      throw new IllegalArgumentException("defaultValues must have length 6");
    }

    kP = new LoggedTunableNumber(namePrefix + " kP", defaultValues[0]);
    kI = new LoggedTunableNumber(namePrefix + " kI", defaultValues[1]);
    kD = new LoggedTunableNumber(namePrefix + " kD", defaultValues[2]);
    kS = new LoggedTunableNumber(namePrefix + " kS", defaultValues[3]);
    kV = new LoggedTunableNumber(namePrefix + " kV", defaultValues[4]);
    kA = new LoggedTunableNumber(namePrefix + " kA", defaultValues[5]);
  }

  public LoggedTunableNumber[] getGainsArray() {
    return new LoggedTunableNumber[] {kP, kI, kD, kS, kV, kA};
  }

  public void ifChanged(int id, GainsConsumer callback) {
    LoggedTunableNumber.ifChanged(
        id,
        gains -> callback.accept(gains[0], gains[1], gains[2], gains[3], gains[4], gains[5]),
        getGainsArray());
    return;
  }

  public void ifPIDChanged(int id, PIDGainsConsumer callback) {
    LoggedTunableNumber.ifChanged(
        id,
        gains -> callback.accept(gains[0], gains[1], gains[2]),
        new LoggedTunableNumber[] {kP, kI, kD});
    return;
  }

  public void ifFeedForwardChanged(int id, FeedForwardGainsConsumer callback) {
    LoggedTunableNumber.ifChanged(
        id,
        gains -> callback.accept(gains[0], gains[1], gains[2]),
        new LoggedTunableNumber[] {kS, kV, kA});
    return;
  }

  public void getValues(GainsConsumer callback) {
    callback.accept(
        kP.getAsDouble(),
        kI.getAsDouble(),
        kD.getAsDouble(),
        kS.getAsDouble(),
        kV.getAsDouble(),
        kA.getAsDouble());
  }

  public void getPIDValues(PIDGainsConsumer callback) {
    callback.accept(kP.getAsDouble(), kI.getAsDouble(), kD.getAsDouble());
  }

  public void getFeedForwardValues(FeedForwardGainsConsumer callback) {
    callback.accept(kS.getAsDouble(), kV.getAsDouble(), kA.getAsDouble());
  }

  @FunctionalInterface
  public interface GainsConsumer {
    void accept(double kP, double kI, double kD, double kS, double kV, double kA);
  }

  @FunctionalInterface
  public interface PIDGainsConsumer {
    void accept(double kP, double kI, double kD);
  }

  @FunctionalInterface
  public interface FeedForwardGainsConsumer {
    void accept(double kS, double kV, double kA);
  }
}
