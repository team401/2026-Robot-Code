package frc.robot.util;

import coppercore.parameter_tools.LoggedTunableNumber;
import coppercore.wpilib_interface.subsystems.motors.MotorIO;

public class LoggedTunablePIDGains {
  LoggedTunableNumber kP;
  LoggedTunableNumber kI;
  LoggedTunableNumber kD;
  LoggedTunableNumber kS;
  LoggedTunableNumber kG;
  LoggedTunableNumber kV;
  LoggedTunableNumber kA;

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
          defaultGains.kG(),
          defaultGains.kV(),
          defaultGains.kA()
        });
  }

  public LoggedTunablePIDGains(String namePrefix, double[] defaultValues) {
    if (defaultValues.length != 7) {
      throw new IllegalArgumentException("defaultValues must have length 7");
    }

    kP = new LoggedTunableNumber(namePrefix + "/kP", defaultValues[0]);
    kI = new LoggedTunableNumber(namePrefix + "/kI", defaultValues[1]);
    kD = new LoggedTunableNumber(namePrefix + "/kD", defaultValues[2]);
    kS = new LoggedTunableNumber(namePrefix + "/kS", defaultValues[3]);
    kG = new LoggedTunableNumber(namePrefix + "/kG", defaultValues[4]);
    kV = new LoggedTunableNumber(namePrefix + "/kV", defaultValues[5]);
    kA = new LoggedTunableNumber(namePrefix + "/kA", defaultValues[6]);
  }

  public LoggedTunableNumber[] getGainsArray() {
    return new LoggedTunableNumber[] {kP, kI, kD, kS, kG, kV, kA};
  }

  public PIDGains getCurrentGains() {
    return new PIDGains(
        kP.getAsDouble(),
        kI.getAsDouble(),
        kD.getAsDouble(),
        kS.getAsDouble(),
        kG.getAsDouble(),
        kV.getAsDouble(),
        kA.getAsDouble());
  }

  public void ifChanged(int id, GainsConsumer callback) {
    LoggedTunableNumber.ifChanged(id, gains -> callback.accept(getCurrentGains()), getGainsArray());
  }

  public void getValues(GainsConsumer callback) {
    callback.accept(getCurrentGains());
  }

  public GainsConsumer getMotorIOApplier(MotorIO motorIO) {
    return pidGains -> pidGains.applyToMotorIO(motorIO);
  }

  public GainsConsumer getMotorIOApplier(MotorIO motorIO, GainsConsumer passThroughConsumer) {
    return pidGains -> {
      pidGains.applyToMotorIO(motorIO);
      passThroughConsumer.accept(pidGains);
    };
  }

  @FunctionalInterface
  public interface GainsConsumer {
    void accept(PIDGains pidGains);
  }
}
