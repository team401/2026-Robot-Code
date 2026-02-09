package frc.robot.util;

import coppercore.parameter_tools.json.helpers.JSONObject;
import coppercore.wpilib_interface.subsystems.motors.profile.MotionProfileConfig;
import coppercore.wpilib_interface.subsystems.motors.profile.MutableMotionProfileConfig;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.units.measure.Velocity;
import java.lang.reflect.Constructor;

public class JSONMotionProfileConfig extends JSONObject<MotionProfileConfig> {
  boolean isMutable;
  AngularVelocity maxVelocity;
  AngularAcceleration maxAcceleration;
  Velocity<AngularAccelerationUnit> maxJerk;
  Per<VoltageUnit, AngularVelocityUnit> voltsPerVelocity;
  Per<VoltageUnit, AngularAccelerationUnit> voltsPerAcceleration;

  public JSONMotionProfileConfig(MotionProfileConfig motionProfileConfig) {
    super(motionProfileConfig);

    this.isMutable = motionProfileConfig instanceof MutableMotionProfileConfig;
    this.maxVelocity = motionProfileConfig.getMaxVelocity();
    this.maxAcceleration = motionProfileConfig.getMaxAcceleration();
    this.maxJerk = motionProfileConfig.getMaxJerk();
    this.voltsPerVelocity = motionProfileConfig.getExpoKv();
    this.voltsPerAcceleration = motionProfileConfig.getExpoKa();
  }

  @Override
  public MotionProfileConfig toJava() {
    if (isMutable) {
      return MotionProfileConfig.mutable(
          maxVelocity, maxAcceleration, maxJerk, voltsPerVelocity, voltsPerAcceleration);
    } else {
      return MotionProfileConfig.immutable(
          maxVelocity, maxAcceleration, maxJerk, voltsPerVelocity, voltsPerAcceleration);
    }
  }

  public static Constructor<JSONMotionProfileConfig> getConstructor() throws NoSuchMethodException {
    return JSONMotionProfileConfig.class.getConstructor(MotionProfileConfig.class);
  }
}
