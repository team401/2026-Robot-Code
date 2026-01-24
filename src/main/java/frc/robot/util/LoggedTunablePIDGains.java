package frc.robot.util;

import coppercore.parameter_tools.LoggedTunableNumber;

public class LoggedTunablePIDGains {
  LoggedTunableNumber kP;
  LoggedTunableNumber kI;
  LoggedTunableNumber kD;
  LoggedTunableNumber kS;
  LoggedTunableNumber kV;
  LoggedTunableNumber kA;
  LoggedTunableNumber kG;

  public LoggedTunablePIDGains(String namePrefix) {
    this(namePrefix, new double[] {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  }

  public LoggedTunablePIDGains(String namePrefix, PIDGains defaultGains) {
    this(
        namePrefix,
        new double[] {
          defaultGains.kP(),
          defaultGains.kI(),
          defaultGains.kD(),
          defaultGains.kS(),
          defaultGains.kV(),
          defaultGains.kA(),
          defaultGains.kG()
        });
  }

  public LoggedTunablePIDGains(String namePrefix, double[] defaultValues) {
    if (defaultValues.length != 7) {
      throw new IllegalArgumentException("defaultValues must have length 7");
    }

    kP = new LoggedTunableNumber(namePrefix + " kP", defaultValues[0]);
    kI = new LoggedTunableNumber(namePrefix + " kI", defaultValues[1]);
    kD = new LoggedTunableNumber(namePrefix + " kD", defaultValues[2]);
    kS = new LoggedTunableNumber(namePrefix + " kS", defaultValues[3]);
    kV = new LoggedTunableNumber(namePrefix + " kV", defaultValues[4]);
    kA = new LoggedTunableNumber(namePrefix + " kA", defaultValues[5]);
    kG = new LoggedTunableNumber(namePrefix + " kG", defaultValues[6]);
  }

  public LoggedTunableNumber[] getGainsArray() {
    return new LoggedTunableNumber[] {kP, kI, kD, kS, kV, kA, kG};
  }

  public PIDGains getCurrentGains() {
    return new PIDGains(
        kP.getAsDouble(),
        kI.getAsDouble(),
        kD.getAsDouble(),
        kS.getAsDouble(),
        kV.getAsDouble(),
        kA.getAsDouble(),
        kG.getAsDouble());
  }

  public void ifChanged(int id, GainsConsumer callback) {
    LoggedTunableNumber.ifChanged(id, gains -> callback.accept(getCurrentGains()), getGainsArray());
  }

  public void getValues(GainsConsumer callback) {
    callback.accept(getCurrentGains());
  }

  @FunctionalInterface
  public interface GainsConsumer {
    void accept(PIDGains pidGains);
  }
}
