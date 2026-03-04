package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import coppercore.wpilib_interface.subsystems.configs.CANDeviceID;
import coppercore.wpilib_interface.subsystems.configs.MechanismConfig;
import coppercore.wpilib_interface.subsystems.configs.MechanismConfig.GravityFeedforwardType;
import coppercore.wpilib_interface.subsystems.motors.profile.MotionProfileConfig;
import coppercore.wpilib_interface.subsystems.sim.CoppercoreSimAdapter;
import coppercore.wpilib_interface.subsystems.sim.DCMotorSimAdapter;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.util.PIDGains;

public class TransferRollerConstants {

  public final Double transferRollerReduction = 1.0; // TODO: find actual value

  // Constant for maximum relative velocity error between the desired motor output and the actual
  // motor output
  public final Double transferRollerMaximumRelativeVelocityError = 0.01;

  public PIDGains transferRollerGains = PIDGains.kPID(10.0, 5.0, 0.0);

  // The important values here are maxAcceleration and maxJerk because it uses
  // a profiled velocity request
  public MotionProfileConfig transferRollerMotionProfileConfig =
      MotionProfileConfig.immutable(
          RotationsPerSecond.zero(),
          RotationsPerSecondPerSecond.of(3000.0),
          RotationsPerSecondPerSecond.of(1000.0).div(Seconds.of(1.0)),
          Volts.zero().div(RotationsPerSecond.of(1)),
          Volts.zero().div(RotationsPerSecondPerSecond.of(1)));

  public MechanismConfig buildMechanismConfig() {
    return MechanismConfig.builder()
        .withName("TransferRoller")
        .withEncoderToMechanismRatio(transferRollerReduction)
        .withMotorToEncoderRatio(1.0)
        .withGravityFeedforwardType(GravityFeedforwardType.STATIC_ELEVATOR)
        .withLeadMotorId(
            new CANDeviceID(
                JsonConstants.robotInfo.CANBus,
                JsonConstants.canBusAssignment.transferRollerKrakenId))
        .build();
  }

  // TODO: Find actual values
  public final Current transferRollerSupplyCurrentLimit = Amps.of(60.0);
  public final Current transferRollerStatorCurrentLimit = Amps.of(40.0);
  public final MomentOfInertia simTransferRollerMOI = KilogramSquareMeters.of(0.00025);

  public final AngularVelocity transferRollerSpinningVelocity = RPM.of(2000);
  public final AngularVelocity transferRollerDeJamVelocity = RPM.of(-2000);

  public final InvertedValue transferRollerMotorDirection = InvertedValue.CounterClockwise_Positive;

  public TalonFXConfiguration buildTalonFXConfigs() {
    return new TalonFXConfiguration()
        .withSlot0(transferRollerGains.toSlot0Config())
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(transferRollerSupplyCurrentLimit)
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(transferRollerStatorCurrentLimit)
                .withStatorCurrentLimitEnable(true))
        .withMotorOutput(new MotorOutputConfigs().withInverted(transferRollerMotorDirection))
        .withMotionMagic(transferRollerMotionProfileConfig.asMotionMagicConfigs());
  }

  public CoppercoreSimAdapter buildTransferRollerSim() {
    return new DCMotorSimAdapter(
        buildMechanismConfig(),
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX44Foc(1),
                simTransferRollerMOI.in(KilogramSquareMeters),
                1 / transferRollerReduction),
            DCMotor.getKrakenX44Foc(1),
            0.0,
            0.0));
  }
}
