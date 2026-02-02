package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import coppercore.wpilib_interface.subsystems.configs.CANDeviceID;
import coppercore.wpilib_interface.subsystems.configs.MechanismConfig;
import coppercore.wpilib_interface.subsystems.configs.MechanismConfig.GravityFeedforwardType;
import coppercore.wpilib_interface.subsystems.sim.ArmSimAdapter;
import coppercore.wpilib_interface.subsystems.sim.CoppercoreSimAdapter;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.Constants.Mode;

public class HoodConstants {
  /**
   * How long the hood motor IO must report a disconnected state before displaying the alert. This
   * should be long enough that momentary disconnects (e.g. from going over the bump) don't trigger
   * the alert, but actual disconnects don't go unnoticed for a long time.
   *
   * <p>A "disconnected state" means that the IO has reported that the motor is not connected. It
   * detects this when the Version signal has not been updated for the past 0.5 seconds, indicating
   * the connection to the motor has been lost. This can indicate a disconnected CAN wire or an
   * electronic failure in the motor itself.
   */
  public final Double disconnectedDebounceTimeSeconds = 1.0;

  /**
   * The angle offset/conversion between the mechanism angle (which must be 0 = center-of-mass
   * horizontal due to Phoenix-6 constraints) and the exit angle of a fuel being shot.
   *
   * <p>90 - exit angle - mechanismAngleToExitAngle = mechanismAngle
   *
   * <p>90 - mechanismAngle - mechanismAngleToExitAngle = exitAngle
   */
  public final Angle mechanismAngleToExitAngle =
      Degrees.of(10.0); // Placeholder. TODO: Analyze CAD for actual measurement.

  public Double hoodKP = 80.0;
  public Double hoodKI = 0.0;
  public Double hoodKD = 20.0;
  public Double hoodKS = 0.0;
  public Double hoodKG = 2.5;
  public Double hoodKV = 0.0;
  public Double hoodKA = 0.1;

  public Double hoodExpoKV = 32.0;
  public Double hoodExpoKA = 32.0;

  public final Double hoodReduction = 20.0; // TODO: Get real value once design finalizes it

  public MechanismConfig buildMechanismConfig() {
    return MechanismConfig.builder()
        .withName("Hood")
        .withEncoderToMechanismRatio(hoodReduction)
        .withGravityFeedforwardType(GravityFeedforwardType.COSINE_ARM)
        .withLeadMotorId(
            new CANDeviceID(
                new CANBus(JsonConstants.robotInfo.canivoreBusName),
                JsonConstants.canBusAssignment.hoodKrakenId))
        .build();
  }

  public final Current hoodSupplyCurrentLimit = Amps.of(60.0);
  public final Current hoodStatorCurrentLimit = Amps.of(80.0);

  public final InvertedValue hoodMotorDirection = InvertedValue.Clockwise_Positive;

  public final Angle minHoodAngle = Degrees.of(10.0);
  public final Angle maxHoodAngle = Degrees.of(30.0);

  public final Voltage homingVoltage = Volts.of(-3.0);
  public final AngularVelocity homingMovementThreshold = DegreesPerSecond.of(2.0);

  public final Time homingMaxUnmovingTime = Seconds.of(5.0);

  public TalonFXConfiguration buildTalonFXConfigs() {
    return new TalonFXConfiguration()
        .withSlot0(
            new Slot0Configs()
                .withKP(hoodKP)
                .withKI(hoodKI)
                .withKD(hoodKD)
                .withKS(hoodKS)
                .withKG(hoodKG)
                .withKV(hoodKV)
                .withKA(hoodKA))
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(hoodSupplyCurrentLimit)
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(hoodStatorCurrentLimit)
                .withStatorCurrentLimitEnable(true))
        .withMotorOutput(
            new MotorOutputConfigs()
                .withInverted(
                    Constants.currentMode == Mode.REAL
                        ? hoodMotorDirection
                        : InvertedValue
                            .CounterClockwise_Positive)) // TODO: Fix motor inverts with single
        // jointed arm sim
        // adapter/MotorIOTalonFXSim
        .withMotionMagic(
            new MotionMagicConfigs()
                .withMotionMagicExpo_kA(hoodExpoKA)
                .withMotionMagicExpo_kV(hoodExpoKV))
        .withFeedback(
            new FeedbackConfigs()
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                .withSensorToMechanismRatio(hoodReduction));
  }

  public final MomentOfInertia simHoodMOI = KilogramSquareMeters.of(0.01);
  public final Distance simHoodArmLength = Inches.of(7.678); // estimate from CAD

  public CoppercoreSimAdapter buildHoodSim() {
    return new ArmSimAdapter(
        buildMechanismConfig(),
        new SingleJointedArmSim(
            DCMotor.getKrakenX44Foc(1),
            hoodReduction,
            simHoodMOI.in(KilogramSquareMeters),
            simHoodArmLength.in(Meters),
            minHoodAngle.in(Radians),
            maxHoodAngle.in(Radians),
            true,
            minHoodAngle.plus(maxHoodAngle).div(2).in(Radians)));
  }
}
