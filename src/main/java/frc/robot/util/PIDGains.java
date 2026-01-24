package frc.robot.util;

public record PIDGains(double kP, double kI, double kD, double kS, double kV, double kA) {
    public double[] asArray() {
      return new double[] {kP, kI, kD, kS, kV, kA};
    }
  }
