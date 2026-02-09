package frc.robot.util;

import coppercore.parameter_tools.LoggedTunableNumber;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.AngularMomentumUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.units.DimensionlessUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.EnergyUnit;
import edu.wpi.first.units.ForceUnit;
import edu.wpi.first.units.FrequencyUnit;
import edu.wpi.first.units.LinearAccelerationUnit;
import edu.wpi.first.units.LinearMomentumUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.MassUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MomentOfInertiaUnit;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.PowerUnit;
import edu.wpi.first.units.ResistanceUnit;
import edu.wpi.first.units.TemperatureUnit;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.TorqueUnit;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.VelocityUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularMomentum;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Energy;
import edu.wpi.first.units.measure.Force;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearMomentum;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularAcceleration;
import edu.wpi.first.units.measure.MutAngularMomentum;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutDimensionless;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutEnergy;
import edu.wpi.first.units.measure.MutForce;
import edu.wpi.first.units.measure.MutFrequency;
import edu.wpi.first.units.measure.MutLinearAcceleration;
import edu.wpi.first.units.measure.MutLinearMomentum;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutMass;
import edu.wpi.first.units.measure.MutMomentOfInertia;
import edu.wpi.first.units.measure.MutPer;
import edu.wpi.first.units.measure.MutPower;
import edu.wpi.first.units.measure.MutResistance;
import edu.wpi.first.units.measure.MutTemperature;
import edu.wpi.first.units.measure.MutTime;
import edu.wpi.first.units.measure.MutTorque;
import edu.wpi.first.units.measure.MutVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.units.measure.Power;
import edu.wpi.first.units.measure.Resistance;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Torque;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;

public class LoggedTunableMeasure<
    MutMeasureType extends MutableMeasure<BaseUnitType, BaseMeasureType, MutMeasureType>,
    BaseMeasureType extends Measure<BaseUnitType>,
    BaseUnitType extends Unit> {

  final MutMeasureType value;
  final LoggedTunableNumber tunableNumber;
  final BaseUnitType displayedUnit;

  public LoggedTunableMeasure(
      String name, MutMeasureType defaultValue, BaseUnitType displayedUnit) {
    this.value = defaultValue;
    this.tunableNumber = new LoggedTunableNumber(name, defaultValue.in(displayedUnit));
    this.displayedUnit = displayedUnit;
  }

  private void updateValue(double newValue) {
    value.mut_replace(newValue, displayedUnit);
  }

  public void forceUpdate() {
    updateValue(tunableNumber.getAsDouble());
  }

  public void ifChanged(
      int id, MeasureConsumer<MutMeasureType, BaseMeasureType, BaseUnitType> callback) {
    LoggedTunableNumber.ifChanged(
        id,
        newValue -> {
          updateValue(newValue[0]);
          callback.accept(value);
        },
        tunableNumber);
  }

  public void ifChanged(MeasureConsumer<MutMeasureType, BaseMeasureType, BaseUnitType> callback) {
    ifChanged(hashCode(), callback);
  }

  protected void checkForUpdate() {
    LoggedTunableNumber.ifChanged(hashCode(), newValue -> updateValue(newValue[0]), tunableNumber);
  }

  public boolean hasChanged(int id) {
    return tunableNumber.hasChanged(id);
  }

  public boolean hasChanged() {
    return hasChanged(hashCode());
  }

  public BaseMeasureType get() {
    checkForUpdate();
    return value.copy();
  }

  public void set(BaseMeasureType newValue) {
    value.mut_replace(newValue);
    tunableNumber.setValue((value.in(displayedUnit)));
  }

  @FunctionalInterface
  public interface MeasureConsumer<
      M extends MutableMeasure<U, B, M>, B extends Measure<U>, U extends Unit> {
    void accept(M newValue);

    default MeasureConsumer<M, B, U> chain(MeasureConsumer<M, B, U> after) {
      return (M newValue) -> {
        this.accept(newValue);
        after.accept(newValue);
      };
    }
  }

  public static class LoggedTunableMeasureFactory<
      M extends MutableMeasure<U, B, M>,
      B extends Measure<U>,
      U extends Unit,
      S extends LoggedTunableMeasure<M, B, U>> {

    LoggedTunableMeasureFactoryFunction<M, B, U, S> factoryFunction;

    public LoggedTunableMeasureFactory(
        LoggedTunableMeasureFactoryFunction<M, B, U, S> factoryFunction) {
      this.factoryFunction = factoryFunction;
    }

    public S of(String name, M defaultValue, U displayedUnit) {
      return factoryFunction.apply(name, defaultValue, displayedUnit);
    }

    public S of(String name, M defaultValue) {
      return of(name, defaultValue, defaultValue.unit());
    }

    public S of(String name, B defaultValue) {
      return of(name, defaultValue, defaultValue.unit());
    }

    @SuppressWarnings("unchecked")
    public S of(String name, B defaultValue, U displayedUnit) {
      return of(name, (M) defaultValue.mutableCopy(), displayedUnit);
    }

    @SuppressWarnings("unchecked")
    public S of(String name, double defaultValue, U displayedUnit) {
      return of(name, (M) displayedUnit.mutable(defaultValue), displayedUnit);
    }

    @FunctionalInterface
    public interface LoggedTunableMeasureFactoryFunction<
        M extends MutableMeasure<U, B, M>,
        B extends Measure<U>,
        U extends Unit,
        S extends LoggedTunableMeasure<M, B, U>> {
      S apply(String name, M defaultValue, U displayedUnit);
    }
  }

  public static class LoggedAngle extends LoggedTunableMeasure<MutAngle, Angle, AngleUnit> {
    public LoggedAngle(String name, MutAngle defaultValue, AngleUnit displayedUnit) {
      super(name, defaultValue, displayedUnit);
    }
  }

  public static class LoggedAngularAcceleration
      extends LoggedTunableMeasure<
          MutAngularAcceleration, AngularAcceleration, AngularAccelerationUnit> {
    public LoggedAngularAcceleration(
        String name, MutAngularAcceleration defaultValue, AngularAccelerationUnit displayedUnit) {
      super(name, defaultValue, displayedUnit);
    }
  }

  public static class LoggedAngularMomentum
      extends LoggedTunableMeasure<MutAngularMomentum, AngularMomentum, AngularMomentumUnit> {
    public LoggedAngularMomentum(
        String name, MutAngularMomentum defaultValue, AngularMomentumUnit displayedUnit) {
      super(name, defaultValue, displayedUnit);
    }
  }

  public static class LoggedAngularVelocity
      extends LoggedTunableMeasure<MutAngularVelocity, AngularVelocity, AngularVelocityUnit> {
    public LoggedAngularVelocity(
        String name, MutAngularVelocity defaultValue, AngularVelocityUnit displayedUnit) {
      super(name, defaultValue, displayedUnit);
    }
  }

  public static class LoggedCurrent extends LoggedTunableMeasure<MutCurrent, Current, CurrentUnit> {
    public LoggedCurrent(String name, MutCurrent defaultValue, CurrentUnit displayedUnit) {
      super(name, defaultValue, displayedUnit);
    }
  }

  public static class LoggedDimensionless
      extends LoggedTunableMeasure<MutDimensionless, Dimensionless, DimensionlessUnit> {
    public LoggedDimensionless(
        String name, MutDimensionless defaultValue, DimensionlessUnit displayedUnit) {
      super(name, defaultValue, displayedUnit);
    }
  }

  public static class LoggedDistance
      extends LoggedTunableMeasure<MutDistance, Distance, DistanceUnit> {
    public LoggedDistance(String name, MutDistance defaultValue, DistanceUnit displayedUnit) {
      super(name, defaultValue, displayedUnit);
    }
  }

  public static class LoggedEnergy extends LoggedTunableMeasure<MutEnergy, Energy, EnergyUnit> {
    public LoggedEnergy(String name, MutEnergy defaultValue, EnergyUnit displayedUnit) {
      super(name, defaultValue, displayedUnit);
    }
  }

  public static class LoggedForce extends LoggedTunableMeasure<MutForce, Force, ForceUnit> {
    public LoggedForce(String name, MutForce defaultValue, ForceUnit displayedUnit) {
      super(name, defaultValue, displayedUnit);
    }
  }

  public static class LoggedFrequency
      extends LoggedTunableMeasure<MutFrequency, Frequency, FrequencyUnit> {
    public LoggedFrequency(String name, MutFrequency defaultValue, FrequencyUnit displayedUnit) {
      super(name, defaultValue, displayedUnit);
    }
  }

  public static class LoggedLinearAcceleration
      extends LoggedTunableMeasure<
          MutLinearAcceleration, LinearAcceleration, LinearAccelerationUnit> {
    public LoggedLinearAcceleration(
        String name, MutLinearAcceleration defaultValue, LinearAccelerationUnit displayedUnit) {
      super(name, defaultValue, displayedUnit);
    }
  }

  public static class LoggedLinearMomentum
      extends LoggedTunableMeasure<MutLinearMomentum, LinearMomentum, LinearMomentumUnit> {
    public LoggedLinearMomentum(
        String name, MutLinearMomentum defaultValue, LinearMomentumUnit displayedUnit) {
      super(name, defaultValue, displayedUnit);
    }
  }

  public static class LoggedLinearVelocity
      extends LoggedTunableMeasure<MutLinearVelocity, LinearVelocity, LinearVelocityUnit> {
    public LoggedLinearVelocity(
        String name, MutLinearVelocity defaultValue, LinearVelocityUnit displayedUnit) {
      super(name, defaultValue, displayedUnit);
    }
  }

  public static class LoggedMass extends LoggedTunableMeasure<MutMass, Mass, MassUnit> {
    public LoggedMass(String name, MutMass defaultValue, MassUnit displayedUnit) {
      super(name, defaultValue, displayedUnit);
    }
  }

  public static class LoggedMomentOfInertia
      extends LoggedTunableMeasure<MutMomentOfInertia, MomentOfInertia, MomentOfInertiaUnit> {
    public LoggedMomentOfInertia(
        String name, MutMomentOfInertia defaultValue, MomentOfInertiaUnit displayedUnit) {
      super(name, defaultValue, displayedUnit);
    }
  }

  public static class LoggedPower extends LoggedTunableMeasure<MutPower, Power, PowerUnit> {
    public LoggedPower(String name, MutPower defaultValue, PowerUnit displayedUnit) {
      super(name, defaultValue, displayedUnit);
    }
  }

  public static class LoggedResistance
      extends LoggedTunableMeasure<MutResistance, Resistance, ResistanceUnit> {
    public LoggedResistance(String name, MutResistance defaultValue, ResistanceUnit displayedUnit) {
      super(name, defaultValue, displayedUnit);
    }
  }

  public static class LoggedTemperature
      extends LoggedTunableMeasure<MutTemperature, Temperature, TemperatureUnit> {
    public LoggedTemperature(
        String name, MutTemperature defaultValue, TemperatureUnit displayedUnit) {
      super(name, defaultValue, displayedUnit);
    }
  }

  public static class LoggedTime extends LoggedTunableMeasure<MutTime, Time, TimeUnit> {
    public LoggedTime(String name, MutTime defaultValue, TimeUnit displayedUnit) {
      super(name, defaultValue, displayedUnit);
    }
  }

  public static class LoggedTorque extends LoggedTunableMeasure<MutTorque, Torque, TorqueUnit> {
    public LoggedTorque(String name, MutTorque defaultValue, TorqueUnit displayedUnit) {
      super(name, defaultValue, displayedUnit);
    }
  }

  public static class LoggedVoltage extends LoggedTunableMeasure<MutVoltage, Voltage, VoltageUnit> {
    public LoggedVoltage(String name, MutVoltage defaultValue, VoltageUnit displayedUnit) {
      super(name, defaultValue, displayedUnit);
    }
  }

  public static LoggedTunableMeasureFactory<MutAngle, Angle, AngleUnit, LoggedAngle> ANGLE =
      new LoggedTunableMeasureFactory<>(LoggedAngle::new);
  public static LoggedTunableMeasureFactory<
          MutAngularAcceleration,
          AngularAcceleration,
          AngularAccelerationUnit,
          LoggedAngularAcceleration>
      ANGULAR_ACCELERATION = new LoggedTunableMeasureFactory<>(LoggedAngularAcceleration::new);
  public static LoggedTunableMeasureFactory<
          MutAngularMomentum, AngularMomentum, AngularMomentumUnit, LoggedAngularMomentum>
      ANGULAR_MOMENTUM = new LoggedTunableMeasureFactory<>(LoggedAngularMomentum::new);
  public static LoggedTunableMeasureFactory<
          MutAngularVelocity, AngularVelocity, AngularVelocityUnit, LoggedAngularVelocity>
      ANGULAR_VELOCITY = new LoggedTunableMeasureFactory<>(LoggedAngularVelocity::new);
  public static LoggedTunableMeasureFactory<MutCurrent, Current, CurrentUnit, LoggedCurrent>
      CURRENT = new LoggedTunableMeasureFactory<>(LoggedCurrent::new);
  public static LoggedTunableMeasureFactory<
          MutDimensionless, Dimensionless, DimensionlessUnit, LoggedDimensionless>
      DIMENSIONLESS = new LoggedTunableMeasureFactory<>(LoggedDimensionless::new);
  public static LoggedTunableMeasureFactory<MutDistance, Distance, DistanceUnit, LoggedDistance>
      DISTANCE = new LoggedTunableMeasureFactory<>(LoggedDistance::new);
  public static LoggedTunableMeasureFactory<MutEnergy, Energy, EnergyUnit, LoggedEnergy> ENERGY =
      new LoggedTunableMeasureFactory<>(LoggedEnergy::new);
  public static LoggedTunableMeasureFactory<MutForce, Force, ForceUnit, LoggedForce> FORCE =
      new LoggedTunableMeasureFactory<>(LoggedForce::new);
  public static LoggedTunableMeasureFactory<MutFrequency, Frequency, FrequencyUnit, LoggedFrequency>
      FREQUENCY = new LoggedTunableMeasureFactory<>(LoggedFrequency::new);
  public static LoggedTunableMeasureFactory<
          MutLinearAcceleration,
          LinearAcceleration,
          LinearAccelerationUnit,
          LoggedLinearAcceleration>
      LINEAR_ACCELERATION = new LoggedTunableMeasureFactory<>(LoggedLinearAcceleration::new);
  public static LoggedTunableMeasureFactory<
          MutLinearMomentum, LinearMomentum, LinearMomentumUnit, LoggedLinearMomentum>
      LINEAR_MOMENTUM = new LoggedTunableMeasureFactory<>(LoggedLinearMomentum::new);
  public static LoggedTunableMeasureFactory<
          MutLinearVelocity, LinearVelocity, LinearVelocityUnit, LoggedLinearVelocity>
      LINEAR_VELOCITY = new LoggedTunableMeasureFactory<>(LoggedLinearVelocity::new);
  public static LoggedTunableMeasureFactory<MutMass, Mass, MassUnit, LoggedMass> MASS =
      new LoggedTunableMeasureFactory<>(LoggedMass::new);
  public static LoggedTunableMeasureFactory<
          MutMomentOfInertia, MomentOfInertia, MomentOfInertiaUnit, LoggedMomentOfInertia>
      MOMENT_OF_INERTIA = new LoggedTunableMeasureFactory<>(LoggedMomentOfInertia::new);
  public static LoggedTunableMeasureFactory<MutPower, Power, PowerUnit, LoggedPower> POWER =
      new LoggedTunableMeasureFactory<>(LoggedPower::new);
  public static LoggedTunableMeasureFactory<
          MutResistance, Resistance, ResistanceUnit, LoggedResistance>
      RESISTANCE = new LoggedTunableMeasureFactory<>(LoggedResistance::new);
  public static LoggedTunableMeasureFactory<
          MutTemperature, Temperature, TemperatureUnit, LoggedTemperature>
      TEMPERATURE = new LoggedTunableMeasureFactory<>(LoggedTemperature::new);
  public static LoggedTunableMeasureFactory<MutTime, Time, TimeUnit, LoggedTime> TIME =
      new LoggedTunableMeasureFactory<>(LoggedTime::new);
  public static LoggedTunableMeasureFactory<MutTorque, Torque, TorqueUnit, LoggedTorque> TORQUE =
      new LoggedTunableMeasureFactory<>(LoggedTorque::new);
  public static LoggedTunableMeasureFactory<MutVoltage, Voltage, VoltageUnit, LoggedVoltage>
      VOLTAGE = new LoggedTunableMeasureFactory<>(LoggedVoltage::new);

  public static class LoggedAngularJerk
      extends LoggedTunableMeasure<
          MutVelocity<AngularAccelerationUnit>,
          Velocity<AngularAccelerationUnit>,
          VelocityUnit<AngularAccelerationUnit>> {
    public LoggedAngularJerk(
        String name,
        MutVelocity<AngularAccelerationUnit> defaultValue,
        VelocityUnit<AngularAccelerationUnit> displayedUnit) {
      super(name, defaultValue, displayedUnit);
    }
  }

  public static class LoggedVoltagePerAngularVelocity
      extends LoggedTunableMeasure<
          MutPer<VoltageUnit, AngularVelocityUnit>,
          Per<VoltageUnit, AngularVelocityUnit>,
          PerUnit<VoltageUnit, AngularVelocityUnit>> {
    public LoggedVoltagePerAngularVelocity(
        String name,
        MutPer<VoltageUnit, AngularVelocityUnit> defaultValue,
        PerUnit<VoltageUnit, AngularVelocityUnit> displayedUnit) {
      super(name, defaultValue, displayedUnit);
    }
  }

  public static class LoggedVoltagePerAngularAcceleration
      extends LoggedTunableMeasure<
          MutPer<VoltageUnit, AngularAccelerationUnit>,
          Per<VoltageUnit, AngularAccelerationUnit>,
          PerUnit<VoltageUnit, AngularAccelerationUnit>> {
    public LoggedVoltagePerAngularAcceleration(
        String name,
        MutPer<VoltageUnit, AngularAccelerationUnit> defaultValue,
        PerUnit<VoltageUnit, AngularAccelerationUnit> displayedUnit) {
      super(name, defaultValue, displayedUnit);
    }
  }

  public static LoggedTunableMeasureFactory<
          MutVelocity<AngularAccelerationUnit>,
          Velocity<AngularAccelerationUnit>,
          VelocityUnit<AngularAccelerationUnit>,
          LoggedAngularJerk>
      ANGULAR_JERK = new LoggedTunableMeasureFactory<>(LoggedAngularJerk::new);
  public static LoggedTunableMeasureFactory<
          MutPer<VoltageUnit, AngularVelocityUnit>,
          Per<VoltageUnit, AngularVelocityUnit>,
          PerUnit<VoltageUnit, AngularVelocityUnit>,
          LoggedVoltagePerAngularVelocity>
      VOLTAGE_PER_ANGULAR_VELOCITY =
          new LoggedTunableMeasureFactory<>(LoggedVoltagePerAngularVelocity::new);
  public static LoggedTunableMeasureFactory<
          MutPer<VoltageUnit, AngularAccelerationUnit>,
          Per<VoltageUnit, AngularAccelerationUnit>,
          PerUnit<VoltageUnit, AngularAccelerationUnit>,
          LoggedVoltagePerAngularAcceleration>
      VOLTAGE_PER_ANGULAR_ACCELERATION =
          new LoggedTunableMeasureFactory<>(LoggedVoltagePerAngularAcceleration::new);
}
