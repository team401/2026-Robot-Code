package frc.robot.util.json;

import coppercore.parameter_tools.json.helpers.JSONObject;
import edu.wpi.first.math.geometry.Rotation2d;
import java.lang.reflect.Constructor;

/** A JSON representation of a Rotation2d object. */
public class JSONRotation2d extends JSONObject<Object> {
  double degrees;

  /**
   * Default constructor for JSON serialization.
   *
   * @param rotation The Rotation2d object to convert to JSON.
   */
  public JSONRotation2d(Rotation2d rotation) {
    super(rotation);
    degrees = rotation.getDegrees();
  }

  @Override
  public Rotation2d toJava() {
    return new Rotation2d(Math.toRadians(degrees));
  }

  /**
   * Gets the constructor of the json wrapper
   *
   * @return the json wrapper constructor
   */
  public static Constructor<JSONRotation2d> getConstructor() throws NoSuchMethodException {
    return JSONRotation2d.class.getConstructor(Rotation2d.class);
  }
}
