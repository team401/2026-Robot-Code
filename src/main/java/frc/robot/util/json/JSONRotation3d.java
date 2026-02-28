package frc.robot.util.json;

import coppercore.parameter_tools.json.helpers.JSONObject;
import edu.wpi.first.math.geometry.Rotation3d;
import java.lang.reflect.Constructor;

// CONSIDER: Using a different representation or naming scheme
// WPILib uses (roll, pitch, yaw) in the constructor, but the getters are (getX, getY, getZ).
// And internal stores the rotations as a quaternion.
/** A JSON representation of a Rotation3d object. Currently uses roll, pitch, yaw representation. */
public class JSONRotation3d extends JSONObject<Object> {
  double roll; // Rotation around the X-axis
  double pitch; // Rotation around the Y-axis
  double yaw; // Rotation around the Z-axis

  /**
   * Default constructor for JSON serialization.
   *
   * @param rotation The Rotation3d object to convert to JSON.
   */
  public JSONRotation3d(Rotation3d rotation) {
    super(rotation);
    roll = Math.toDegrees(rotation.getX());
    pitch = Math.toDegrees(rotation.getY());
    yaw = Math.toDegrees(rotation.getZ());
  }

  @Override
  public Rotation3d toJava() {
    return new Rotation3d(Math.toRadians(roll), Math.toRadians(pitch), Math.toRadians(yaw));
  }

  /**
   * Gets the constructor of the json wrapper
   *
   * @return the json wrapper constructor
   */
  public static Constructor<JSONRotation3d> getConstructor() throws NoSuchMethodException {
    return JSONRotation3d.class.getConstructor(Rotation3d.class);
  }
}
