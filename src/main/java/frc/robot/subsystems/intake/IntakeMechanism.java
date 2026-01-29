package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import coppercore.wpilib_interface.subsystems.configs.CANDeviceID;
import coppercore.wpilib_interface.subsystems.configs.MechanismConfig;
import coppercore.wpilib_interface.subsystems.configs.MechanismConfig.GravityFeedforwardType;
import coppercore.wpilib_interface.subsystems.motors.MotorIO;
import coppercore.wpilib_interface.subsystems.motors.MotorInputsAutoLogged;
import coppercore.wpilib_interface.subsystems.motors.talonfx.MotorIOTalonFX;
import coppercore.wpilib_interface.subsystems.motors.talonfx.MotorIOTalonFXPositionSim;
import coppercore.wpilib_interface.subsystems.motors.talonfx.MotorIOTalonFXSim;
import coppercore.wpilib_interface.subsystems.sim.ArmSimAdapter;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.constants.JsonConstants;
import frc.robot.util.DiskSim;
import frc.robot.util.DiskSimAdapter;
import org.littletonrobotics.junction.Logger;

public class IntakeMechanism {

  private MotorIO pivotIO;
  private MotorIO rollersIO;

  private MotorInputsAutoLogged pivotInputs;
  private MotorInputsAutoLogged rollersInputs;

  public static MechanismConfig getPivotMechanismConfig() {
    return MechanismConfig.builder()
        .withName("Intake Pivot Mechanism")
        .withGravityFeedforwardType(GravityFeedforwardType.COSINE_ARM)
        .withLeadMotorId(
            new CANDeviceID(
                JsonConstants.canConstants.canivoreBus,
                JsonConstants.canConstants.intakePivotMotorId))
        .build();
  }

  public static MechanismConfig getRollersMechanismConfig() {
    return MechanismConfig.builder()
        .withName("Intake Rollers Mechanism")
        .withGravityFeedforwardType(GravityFeedforwardType.STATIC_ELEVATOR)
        .withLeadMotorId(
            new CANDeviceID(
                JsonConstants.canConstants.canivoreBus,
                JsonConstants.canConstants.intakeRollersMotorId))
        .build();
  }

  public IntakeMechanism(MotorIO pivotIO, MotorIO rollersIO) {
    this.pivotIO = pivotIO;
    this.rollersIO = rollersIO;

    this.pivotInputs = new MotorInputsAutoLogged();
    this.rollersInputs = new MotorInputsAutoLogged();
  }

  public static TalonFXConfiguration getPivotTalonFXMotorConfig() {
    TalonFXConfiguration config =
        new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimitEnable(true)
                    .withStatorCurrentLimit(Amps.of(80.0)))
            .withSlot0(
                new Slot0Configs()
                    .withGravityType(GravityTypeValue.Arm_Cosine)
                    .withKS(0.0)
                    .withKV(0.0)
                    .withKA(0.0)
                    .withKG(0.0)
                    .withKP(0.5)
                    .withKI(0.0)
                    .withKD(0.1));
    // Configure motor settings here
    return config;
  }

  public static TalonFXConfiguration getRollersTalonFXMotorConfig() {
    TalonFXConfiguration config =
        new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimitEnable(true)
                    .withStatorCurrentLimit(Amps.of(80.0)))
            .withSlot0(
                new Slot0Configs()
                    .withKS(0.0)
                    .withKV(0.0)
                    .withKA(0.0)
                    .withKG(10.0)
                    .withKP(20.0)
                    .withKI(10.0)
                    .withKD(10.0));
    // Configure motor settings here
    return config;
  }

  public static MotorIO createPivotTalonFXMotorIO() {
    return new MotorIOTalonFX(getPivotMechanismConfig(), getPivotTalonFXMotorConfig());
  }

  public static MotorIO createRollersTalonFXMotorIO() {
    return new MotorIOTalonFX(getRollersMechanismConfig(), getRollersTalonFXMotorConfig());
  }

  public static MotorIO createPivotTalonFXSimulationMotorIO() {
    MechanismConfig mechanismConfig = getPivotMechanismConfig();
    return new MotorIOTalonFXSim(
        mechanismConfig,
        getPivotTalonFXMotorConfig(),
        new ArmSimAdapter(
            mechanismConfig,
            new SingleJointedArmSim(
                DCMotor.getKrakenX60Foc(1),
                1.0,
                JsonConstants.intakeConstants.pivotInertia.in(Units.KilogramSquareMeters),
                JsonConstants.intakeConstants.armLengthMeters,
                JsonConstants.intakeConstants.minPivotAngleRadians,
                JsonConstants.intakeConstants.maxPivotAngleRadians,
                false,
                JsonConstants.intakeConstants.pivotStartingAngleRadians)));
  }

  public static MotorIO createRollersTalonFXSimulationMotorIO() {
    MechanismConfig mechanismConfig = getRollersMechanismConfig();
    return new MotorIOTalonFXPositionSim(
        mechanismConfig,
        getRollersTalonFXMotorConfig(),
        new DiskSimAdapter(
            mechanismConfig,
            new DiskSim(
                DCMotor.getKrakenX44(1),
                1,
                JsonConstants.intakeConstants.rollersInertia.in(Units.KilogramSquareMeters),
                0)));
  }

  public void setTargetPivotAngle(Angle angle) {
    pivotIO.controlToPositionUnprofiled(angle);
  }

  public void setTargetPivotAngle(double angleRadians) {
    setTargetPivotAngle(Units.Radians.of(angleRadians));
  }

  public void runRollers(double speedRPM) {
    rollersIO.controlToVelocityUnprofiled(Units.RPM.of(speedRPM));
  }

  public void periodic() {
    pivotIO.updateInputs(pivotInputs);
    rollersIO.updateInputs(rollersInputs);

    Logger.processInputs("intake/pivot/inputs", pivotInputs);
    Logger.processInputs("intake/rollers/inputs", rollersInputs);
  }

  public void testPeriodic() {
    return;
  }
}
