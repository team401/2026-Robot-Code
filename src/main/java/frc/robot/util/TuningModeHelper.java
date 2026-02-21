package frc.robot.util;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import coppercore.wpilib_interface.subsystems.motors.MotorIO;
import coppercore.wpilib_interface.subsystems.motors.profile.MotionProfileConfig;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.LoggedTunableMeasure.LoggedAngle;
import frc.robot.util.LoggedTunableMeasure.LoggedAngularVelocity;
import frc.robot.util.LoggedTunableMeasure.LoggedCurrent;
import frc.robot.util.LoggedTunableMeasure.LoggedVoltage;
import java.util.Arrays;
import java.util.EnumMap;
import java.util.List;
import java.util.function.Consumer;

public class TuningModeHelper<TestModeEnum extends Enum<TestModeEnum> & TestModeDescription> {

  private EnumMap<TestModeEnum, TuningMode> tuningModes;
  private TuningMode currentTestMode = null;

  public TuningModeHelper(Class<TestModeEnum> testModeEnumClass) {
    tuningModes = new EnumMap<>(testModeEnumClass);
  }

  public TuningModeHelper<TestModeEnum> addTuningMode(
      TestModeEnum testMode, TuningMode tuningMode) {
    tuningModes.put(testMode, tuningMode);
    return this;
  }

  public void testPeriodic(TestModeEnum testMode) {

    var tuningMode = tuningModes.get(testMode);

    if (tuningMode != currentTestMode) {
      // Exit the current mode
      if (currentTestMode != null) {
        currentTestMode.exit();
      }
      // Enter the new mode
      if (tuningMode != null) {
        tuningMode.enter();
      }
      currentTestMode = tuningMode;
    }

    if (tuningMode != null) {
      tuningMode.periodic();
    }
  }

  @SafeVarargs
  public final TuningModeHelper<TestModeEnum> addMotorTuningModes(
      TunableMotor motor, MotorTuningMode<TestModeEnum>... motorTuningModes) {
    for (MotorTuningMode<TestModeEnum> motorTuningMode : motorTuningModes) {
      addTuningMode(motorTuningMode.testMode, motor.createTuningMode(motorTuningMode.controlMode));
    }
    return this;
  }

  public record MotorTuningMode<T extends Enum<T>>(T testMode, ControlMode controlMode) {
    public static <T extends Enum<T>> MotorTuningMode<T> of(T testMode, ControlMode controlMode) {
      return new MotorTuningMode<>(testMode, controlMode);
    }
  }

  public record TuningMode(Runnable enterAction, Runnable periodicAction, Runnable exitAction) {

    public void enter() {
      if (enterAction != null) {
        enterAction.run();
      }
    }

    public void periodic() {
      if (periodicAction != null) {
        periodicAction.run();
      }
    }

    public void exit() {
      if (exitAction != null) {
        exitAction.run();
      }
    }

    public static TuningMode of(
        Runnable enterAction, Runnable periodicAction, Runnable exitAction) {
      return new TuningMode(enterAction, periodicAction, exitAction);
    }

    public static TuningMode simple(Runnable periodicAction) {
      return new TuningMode(() -> {}, periodicAction, () -> {});
    }

    public static TuningMode empty() {
      return new TuningMode(() -> {}, () -> {}, () -> {});
    }
  }

  public enum ControlMode {
    OPEN_LOOP_VOLTAGE,
    OPEN_LOOP_CURRENT,
    CLOSED_LOOP,
    BRAKE_MODE,
    COAST_MODE,
    NEUTRAL_MODE,
    PHOENIX_TUNING,
    NONE
  }

  public static class TunableMotor {

    private MotorIO leadMotorIO;
    private List<MotorIO> followerMotorIOs;
    private TunableMotorConfiguration configuration;

    private LoggedVoltage openLoopVoltage = null;
    private LoggedCurrent openLoopCurrent = null;
    private LoggedTunablePIDGains closedLoopPIDGains = null;
    private LoggedTunableMotionProfile closedLoopMotionProfile = null;
    private LoggedAngle closedLoopPositionTarget = null;
    private LoggedAngularVelocity closedLoopVelocityTarget = null;

    public TunableMotor(
        TunableMotorConfiguration configuration,
        String prefix,
        MotorIO leadMotor,
        MotorIO... followerMotorIOs) {
      this.leadMotorIO = leadMotor;
      this.followerMotorIOs = Arrays.asList(followerMotorIOs);
      this.configuration = configuration;

      if (configuration.voltageTuning) {
        openLoopVoltage =
            LoggedTunableMeasure.VOLTAGE.of(prefix + "/VoltageTuning", Volts.zero(), Volts, true);
      }

      if (configuration.currentTuning) {
        openLoopCurrent =
            LoggedTunableMeasure.CURRENT.of(prefix + "/CurrentTuning", Amps.zero(), Amps, true);
      }

      if (configuration.hasClosedLoopTuning) {
        closedLoopPIDGains =
            new LoggedTunablePIDGains(prefix + "/PIDGains", configuration.defaultPIDGains);

        if (configuration.profileType != ProfileType.UNPROFILED) {
          closedLoopMotionProfile =
              new LoggedTunableMotionProfile(
                  prefix + "/MotionProfile",
                  configuration.defaultMotionProfileConfig,
                  configuration.useUnitInLoggedTunablePaths);
        }

        if (configuration.closedLoopType == ClosedLoopType.POSITION) {
          AngleUnit loggedUnit = configuration.tunableAngleUnit;
          if (loggedUnit == null) {
            loggedUnit = configuration.initialPositionSetpoint.unit();
          }
          closedLoopPositionTarget =
              LoggedTunableMeasure.ANGLE.of(
                  prefix + "/PositionTarget",
                  configuration.initialPositionSetpoint,
                  loggedUnit,
                  configuration.useUnitInLoggedTunablePaths);
        } else if (configuration.closedLoopType == ClosedLoopType.VELOCITY) {
          AngularVelocityUnit loggedUnit = configuration.tunableAngularVelocityUnit;
          if (loggedUnit == null) {
            loggedUnit = configuration.initialVelocitySetpoint.unit();
          }
          closedLoopVelocityTarget =
              LoggedTunableMeasure.ANGULAR_VELOCITY.of(
                  prefix + "/VelocityTarget",
                  configuration.initialVelocitySetpoint,
                  loggedUnit,
                  configuration.useUnitInLoggedTunablePaths);
        }
      }
    }

    public void runBrakeMode() {
      leadMotorIO.controlBrake();
    }

    public void runCoastMode() {
      leadMotorIO.controlCoast();
    }

    public void runNeutralMode() {
      leadMotorIO.controlNeutral();
    }

    public void runOpenLoopVoltageTuning() {
      if (!configuration.voltageTuning) {
        return;
      }

      if (openLoopVoltage != null) {
        Voltage voltage = openLoopVoltage.get();
        leadMotorIO.controlOpenLoopVoltage(voltage);
      }
    }

    public void runOpenLoopCurrentTuning() {
      if (!configuration.currentTuning) {
        return;
      }

      if (openLoopCurrent != null) {
        Current current = openLoopCurrent.get();
        leadMotorIO.controlOpenLoopCurrent(current);
      }
    }

    private void applyPIDGainsToMotors(PIDGains pidGains) {
      configuration.onPIDGainsChanged.accept(pidGains);
      pidGains.applyToMotorIO(leadMotorIO);
      followerMotorIOs.forEach(motorIO -> pidGains.applyToMotorIO(motorIO));
    }

    private void applyMotionProfileToMotors(MotionProfileConfig profileConfig) {
      configuration.onMotionProfileConfigChanged.accept(profileConfig);
      leadMotorIO.setProfileConstraints(profileConfig);
      followerMotorIOs.forEach(motorIO -> motorIO.setProfileConstraints(profileConfig));
    }

    public void runClosedLoopTuning() {

      if (!configuration.hasClosedLoopTuning) {
        return;
      }

      // Closed Loop Tuning
      if (closedLoopPIDGains != null) {
        closedLoopPIDGains.ifChanged(hashCode(), this::applyPIDGainsToMotors);
      }

      if (configuration.profileType != ProfileType.UNPROFILED) {
        if (closedLoopMotionProfile != null) {
          closedLoopMotionProfile.ifChanged(hashCode(), this::applyMotionProfileToMotors);
        }
      }

      if (configuration.closedLoopType == ClosedLoopType.POSITION) {
        runClosedLoopPositionTuning();
      } else if (configuration.closedLoopType == ClosedLoopType.VELOCITY) {
        runClosedLoopVelocityTuning();
      }
    }

    private void runClosedLoopPositionTuning() {
      Angle targetAngle = closedLoopPositionTarget.get();

      if (configuration.profileType == ProfileType.UNPROFILED) {
        leadMotorIO.controlToPositionUnprofiled(targetAngle);
      } else if (configuration.profileType == ProfileType.PROFILED) {
        leadMotorIO.controlToPositionProfiled(targetAngle);
      } else if (configuration.profileType == ProfileType.EXPO_PROFILED) {
        leadMotorIO.controlToPositionExpoProfiled(targetAngle);
      }
    }

    private void runClosedLoopVelocityTuning() {
      AngularVelocity targetVelocity = closedLoopVelocityTarget.get();

      if (configuration.profileType == ProfileType.UNPROFILED) {
        leadMotorIO.controlToVelocityUnprofiled(targetVelocity);
      } else if (configuration.profileType == ProfileType.PROFILED) {
        leadMotorIO.controlToVelocityProfiled(targetVelocity);
      }
    }

    private boolean canRunControlMode(ControlMode controlMode) {
      switch (controlMode) {
        case OPEN_LOOP_VOLTAGE:
          return configuration.voltageTuning;
        case OPEN_LOOP_CURRENT:
          return configuration.currentTuning;
        case CLOSED_LOOP:
          return configuration.hasClosedLoopTuning;
        case BRAKE_MODE:
        case COAST_MODE:
        case NEUTRAL_MODE:
          return true;
        case PHOENIX_TUNING:
          return configuration.phoenixTuning;
        case NONE:
          return true;
        default:
          return false;
      }
    }

    public TuningMode createTuningMode(ControlMode controlMode) {

      if (controlMode == null) {
        throw new IllegalArgumentException("Control mode cannot be null for tuning mode.");
      }

      if (!canRunControlMode(controlMode)) {
        throw new IllegalStateException(
            "Control mode " + controlMode + " is not enabled in the configuration.");
      }

      switch (controlMode) {
        case OPEN_LOOP_VOLTAGE:
          return TuningMode.of(() -> {}, () -> runOpenLoopVoltageTuning(), () -> runNeutralMode());
        case OPEN_LOOP_CURRENT:
          return TuningMode.of(() -> {}, () -> runOpenLoopCurrentTuning(), () -> runNeutralMode());
        case CLOSED_LOOP:
          return TuningMode.of(() -> {}, () -> runClosedLoopTuning(), () -> runNeutralMode());
        case BRAKE_MODE:
          return TuningMode.simple(() -> runBrakeMode());
        case COAST_MODE:
          return TuningMode.simple(() -> runCoastMode());
        case NEUTRAL_MODE:
          return TuningMode.simple(() -> runNeutralMode());
        case PHOENIX_TUNING:
          return TuningMode.empty();
        case NONE:
          return TuningMode.empty();
        default:
          throw new IllegalArgumentException("Unsupported control mode for tuning mode.");
      }
    }
  }

  protected enum ClosedLoopType {
    POSITION,
    VELOCITY
  }

  protected enum ProfileType {
    UNPROFILED,
    PROFILED,
    EXPO_PROFILED
  }

  // TODO: Look into if this is a pattern that some type of annotation would be able to generate?
  /**
   * TunableMotorConfiguration is a configuration class for creating TunableMotors. It allows you to
   * specify which control modes you want to be able to tune, as well as default values and
   * callbacks for when PID gains or motion profile configs are changed. Use the builder pattern to
   * create a configuration, then pass it to the TunableMotor constructor. Note the initial unit
   * will affect the logged tunable paths. So if you want to be able to change the initial unit
   * without worrying about the name set the logging unit to what you want the logged unit to be, Or
   * just disable using unit name in the logged tunable paths by using
   */
  public static class TunableMotorConfiguration {
    protected boolean voltageTuning = false;
    protected boolean currentTuning = false;
    protected boolean hasClosedLoopTuning = false;
    protected ClosedLoopType closedLoopType = ClosedLoopType.POSITION;
    protected ProfileType profileType = ProfileType.UNPROFILED;
    protected boolean phoenixTuning = false;
    protected boolean useUnitInLoggedTunablePaths = true;

    // Default Values
    protected Angle initialPositionSetpoint = Radians.zero();
    protected AngularVelocity initialVelocitySetpoint = RadiansPerSecond.zero();
    protected PIDGains defaultPIDGains = new PIDGains(0, 0, 0, 0, 0, 0, 0);
    protected MotionProfileConfig defaultMotionProfileConfig =
        LoggedTunableMotionProfile.defaultMotionProfileConfig;

    protected AngleUnit tunableAngleUnit = null;
    protected AngularVelocityUnit tunableAngularVelocityUnit = null;

    protected Consumer<PIDGains> onPIDGainsChanged = pidGains -> {};
    protected Consumer<MotionProfileConfig> onMotionProfileConfigChanged = profileConfig -> {};

    private TunableMotorConfiguration() {}

    public static TunableMotorConfiguration builder() {
      return new TunableMotorConfiguration();
    }

    public static TunableMotorConfiguration defaultConfiguration() {
      return builder().withVoltageTuning(true).withCurrentTuning(true).withPhoenixTuning(true);
    }

    public TunableMotorConfiguration withVoltageTuning(boolean voltageTuning) {
      this.voltageTuning = voltageTuning;
      return this;
    }

    public TunableMotorConfiguration withCurrentTuning(boolean currentTuning) {
      this.currentTuning = currentTuning;
      return this;
    }

    public TunableMotorConfiguration withoutClosedLoopTuning() {
      this.hasClosedLoopTuning = false;
      return this;
    }

    public TunableMotorConfiguration withPositionTuning() {
      this.closedLoopType = ClosedLoopType.POSITION;
      this.hasClosedLoopTuning = true;
      return this;
    }

    public TunableMotorConfiguration withVelocityTuning() {
      this.closedLoopType = ClosedLoopType.VELOCITY;
      this.hasClosedLoopTuning = true;
      return this;
    }

    public TunableMotorConfiguration unprofiled() {
      this.profileType = ProfileType.UNPROFILED;
      return this;
    }

    public TunableMotorConfiguration profiled() {
      this.profileType = ProfileType.PROFILED;
      return this;
    }

    public TunableMotorConfiguration expoProfiled() {
      this.profileType = ProfileType.EXPO_PROFILED;
      return this;
    }

    public TunableMotorConfiguration withPhoenixTuning(boolean phoenixTuning) {
      this.phoenixTuning = phoenixTuning;
      return this;
    }

    public TunableMotorConfiguration withInitialPositionSetpoint(Angle initialPositionSetpoint) {
      this.initialPositionSetpoint = initialPositionSetpoint;
      return this;
    }

    public TunableMotorConfiguration withInitialVelocitySetpoint(
        AngularVelocity initialVelocitySetpoint) {
      this.initialVelocitySetpoint = initialVelocitySetpoint;
      return this;
    }

    public TunableMotorConfiguration withDefaultPIDGains(PIDGains defaultPIDGains) {
      this.defaultPIDGains = defaultPIDGains;
      return this;
    }

    public TunableMotorConfiguration withDefaultMotionProfileConfig(
        MotionProfileConfig defaultMotionProfileConfig) {
      this.defaultMotionProfileConfig = defaultMotionProfileConfig;
      return this;
    }

    public TunableMotorConfiguration onPIDGainsChanged(Consumer<PIDGains> onPIDGainsChanged) {
      this.onPIDGainsChanged = onPIDGainsChanged;
      return this;
    }

    public TunableMotorConfiguration onMotionProfileConfigChanged(
        Consumer<MotionProfileConfig> onMotionProfileConfigChanged) {
      this.onMotionProfileConfigChanged = onMotionProfileConfigChanged;
      return this;
    }

    public TunableMotorConfiguration withTunableAngleUnit(AngleUnit tunableAngleUnit) {
      this.tunableAngleUnit = tunableAngleUnit;
      return this;
    }

    public TunableMotorConfiguration withTunableAngularVelocityUnit(
        AngularVelocityUnit tunableAngularVelocityUnit) {
      this.tunableAngularVelocityUnit = tunableAngularVelocityUnit;
      return this;
    }

    public TunableMotorConfiguration withUseUnitInLoggedTunablePaths(
        boolean useUnitInLoggedTunablePaths) {
      this.useUnitInLoggedTunablePaths = useUnitInLoggedTunablePaths;
      return this;
    }

    private void validate() {
      if (closedLoopType == ClosedLoopType.VELOCITY && profileType == ProfileType.EXPO_PROFILED) {
        throw new IllegalArgumentException(
            "Exponential profiling is not supported for velocity control.");
      }
    }

    public TunableMotorConfiguration copy() {
      TunableMotorConfiguration copy = new TunableMotorConfiguration();
      copy.voltageTuning = this.voltageTuning;
      copy.currentTuning = this.currentTuning;
      copy.hasClosedLoopTuning = this.hasClosedLoopTuning;
      copy.closedLoopType = this.closedLoopType;
      copy.profileType = this.profileType;
      copy.phoenixTuning = this.phoenixTuning;
      copy.initialPositionSetpoint = this.initialPositionSetpoint;
      copy.initialVelocitySetpoint = this.initialVelocitySetpoint;
      copy.defaultPIDGains = this.defaultPIDGains;
      copy.defaultMotionProfileConfig = this.defaultMotionProfileConfig;
      copy.onPIDGainsChanged = this.onPIDGainsChanged;
      copy.onMotionProfileConfigChanged = this.onMotionProfileConfigChanged;
      copy.tunableAngleUnit = this.tunableAngleUnit;
      copy.tunableAngularVelocityUnit = this.tunableAngularVelocityUnit;
      copy.useUnitInLoggedTunablePaths = this.useUnitInLoggedTunablePaths;
      return copy;
    }

    public TunableMotor build(String prefix, MotorIO leadMotorIO, MotorIO... followerMotorIOs) {
      validate();
      return new TunableMotor(this.copy(), prefix, leadMotorIO, followerMotorIOs);
    }
  }
}
