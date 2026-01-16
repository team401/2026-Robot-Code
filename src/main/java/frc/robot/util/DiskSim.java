package frc.robot.util;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;

public class DiskSim extends LinearSystemSim<N2, N1, N2> {

  private final DCMotor m_gearbox;
  private final double m_gearing;

  public static LinearSystem<N2, N1, N2> createDiskPlant(
      DCMotor gearbox, double gearing, double momentOfInertiaKgM2) {
    // Might need to manually calculate the LinearSystem if this doesn't work as expected
    return LinearSystemId.createSingleJointedArmSystem(gearbox, momentOfInertiaKgM2, gearing);
  }

  public DiskSim(
      LinearSystem<N2, N1, N2> plant,
      DCMotor gearbox,
      double gearing,
      double startingAngleRads,
      double... measurementStdDevs) {
    super(plant, measurementStdDevs);
    m_gearbox = gearbox;
    m_gearing = gearing;

    setState(startingAngleRads, 0.0);
  }

  public DiskSim(
      DCMotor gearbox,
      double gearing,
      double momentOfInertiaKgM2,
      double startingAngleRads,
      double... measurementStdDevs) {
    this(
        DiskSim.createDiskPlant(gearbox, gearing, momentOfInertiaKgM2),
        gearbox,
        gearing,
        startingAngleRads,
        measurementStdDevs);
  }

  public final void setState(double angleRadians, double velocityRadPerSec) {
    setState(VecBuilder.fill(angleRadians, velocityRadPerSec));
  }

  public double getAngleRads() {
    return getOutput(0);
  }

  public double getVelocityRadPerSec() {
    return getOutput(1);
  }

  public double getCurrentDrawAmps() {
    // Reductions are greater than 1, so a reduction of 10:1 would mean the motor is
    // spinning 10x faster than the output
    var motorVelocity = m_x.get(1, 0) * m_gearing;
    return m_gearbox.getCurrent(motorVelocity, m_u.get(0, 0)) * Math.signum(m_u.get(0, 0));
  }
}
