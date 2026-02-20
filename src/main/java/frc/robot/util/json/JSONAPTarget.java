package frc.robot.util.json;

import com.therekrab.autopilot.APTarget;
import coppercore.parameter_tools.json.helpers.JSONObject;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import java.lang.reflect.Constructor;

public class JSONAPTarget extends JSONObject<APTarget> {
  protected Pose2d reference;

  protected Rotation2d entryAngle;
  protected double velocity;
  protected Distance rotationRadius;

  public JSONAPTarget(APTarget target) {
    super(target);
    this.reference = target.getReference();
    this.entryAngle = target.getEntryAngle().orElse(null);
    this.velocity = target.getVelocity();
    this.rotationRadius = target.getRotationRadius().orElse(null);
  }

  @Override
  public APTarget toJava() {
    if (reference == null) {
      reference = new Pose2d();
    }

    var target = new APTarget(reference);

    if (entryAngle != null) {
      target = target.withEntryAngle(entryAngle);
    }

    if (velocity != 0.0) {
      target = target.withVelocity(velocity);
    }

    if (rotationRadius != null) {
      target = target.withRotationRadius(rotationRadius);
    }

    return target;
  }

  /**
   * Gets the constructor of the json wrapper
   *
   * @return the json wrapper constructor
   */
  public static Constructor<JSONAPTarget> getConstructor() throws NoSuchMethodException {
    return JSONAPTarget.class.getConstructor(APTarget.class);
  }
}
