package frc.robot.util.json;

import coppercore.parameter_tools.json.helpers.JSONObject;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.lang.reflect.Constructor;

/** A JSON representation of a Pose2d object. */
public class JSONTransform2d extends JSONObject<Transform2d> {
  Rotation2d rotation;
  Translation2d translation;

  /**
   * Default constructor for JSON serialization.
   *
   * @param transform The Transform2d object to convert to JSON.
   */
  public JSONTransform2d(Transform2d transform) {
    super(transform);
    rotation = transform.getRotation();
    translation = transform.getTranslation();
  }

  @Override
  public Transform2d toJava() {
    return new Transform2d(translation, rotation);
  }

  /**
   * Gets the constructor of the json wrapper
   *
   * @return the json wrapper constructor
   */
  public static Constructor<JSONTransform2d> getConstructor() throws NoSuchMethodException {
    return JSONTransform2d.class.getConstructor(Transform2d.class);
  }
}
