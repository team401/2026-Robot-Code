package frc.robot.util.ts;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * Registers WPILib-style geometry methods (plus, minus, transform_by, etc.) with PythonGenerator so
 * they appear on the generated dataclasses in auto_action.py.
 *
 * <p>Call {@link #registerAll()} before {@code PythonGenerator.generateForClasses(...)}.
 */
public final class PythonGeometryMethods {

  private PythonGeometryMethods() {}

  public static void registerAll() {
    registerRotation2dMethods();
    registerTranslation2dMethods();
    registerPose2dMethods();
    registerTransform2dMethods();
    registerRotation3dMethods();
    registerTranslation3dMethods();
    registerPose3dMethods();
    registerTransform3dMethods();
  }

  // =========================================================================
  // Rotation2d
  // =========================================================================

  private static void registerRotation2dMethods() {
    PythonGenerator.registerExtraMethods(
        Rotation2d.class,
        // properties
        m(
            """
    @property
    def radians(self) -> float:
        return _math.radians(self.degrees)"""),
        m(
            """
    @property
    def cos(self) -> float:
        return _math.cos(self.radians)"""),
        m(
            """
    @property
    def sin(self) -> float:
        return _math.sin(self.radians)"""),
        // constructors
        m(
            """
    @staticmethod
    def from_radians(radians: float) -> Rotation2d:
        return Rotation2d(degrees=_math.degrees(radians))"""),
        // operators
        m(
            """
    def plus(self, other: Rotation2d) -> Rotation2d:
        return Rotation2d(degrees=self.degrees + other.degrees)"""),
        m(
            """
    def minus(self, other: Rotation2d) -> Rotation2d:
        return Rotation2d(degrees=self.degrees - other.degrees)"""),
        m(
            """
    def unary_minus(self) -> Rotation2d:
        return Rotation2d(degrees=-self.degrees)"""),
        m(
            """
    def rotate_by(self, other: Rotation2d) -> Rotation2d:
        return self.plus(other)"""),
        // dunder operators
        m(
            """
    def __neg__(self) -> Rotation2d:
        return self.unary_minus()"""),
        m(
            """
    def __add__(self, other: Rotation2d) -> Rotation2d:
        return self.plus(other)"""),
        m(
            """
    def __sub__(self, other: Rotation2d) -> Rotation2d:
        return self.minus(other)"""));
  }

  // =========================================================================
  // Translation2d
  // =========================================================================

  private static void registerTranslation2dMethods() {
    PythonGenerator.registerExtraMethods(
        Translation2d.class,
        // properties
        m(
            """
    @property
    def norm(self) -> float:
        return _math.hypot(self.x, self.y)"""),
        // operators
        m(
            """
    def plus(self, other: Translation2d) -> Translation2d:
        return Translation2d(x=self.x + other.x, y=self.y + other.y)"""),
        m(
            """
    def minus(self, other: Translation2d) -> Translation2d:
        return Translation2d(x=self.x - other.x, y=self.y - other.y)"""),
        m(
            """
    def unary_minus(self) -> Translation2d:
        return Translation2d(x=-self.x, y=-self.y)"""),
        m(
            """
    def times(self, scalar: float) -> Translation2d:
        return Translation2d(x=self.x * scalar, y=self.y * scalar)"""),
        m(
            """
    def div(self, scalar: float) -> Translation2d:
        return Translation2d(x=self.x / scalar, y=self.y / scalar)"""),
        m(
            """
    def rotate_by(self, rotation: Rotation2d) -> Translation2d:
        c = rotation.cos
        s = rotation.sin
        return Translation2d(x=self.x * c - self.y * s, y=self.x * s + self.y * c)"""),
        m(
            """
    def distance(self, other: Translation2d) -> float:
        return self.minus(other).norm"""),
        // dunder operators
        m(
            """
    def __neg__(self) -> Translation2d:
        return self.unary_minus()"""),
        m(
            """
    def __add__(self, other: Translation2d) -> Translation2d:
        return self.plus(other)"""),
        m(
            """
    def __sub__(self, other: Translation2d) -> Translation2d:
        return self.minus(other)"""),
        m(
            """
    def __mul__(self, scalar: float) -> Translation2d:
        return self.times(scalar)"""),
        m(
            """
    def __truediv__(self, scalar: float) -> Translation2d:
        return self.div(scalar)"""),
        // conversions
        m(
            """
    def to_pose2d(self, rotation: Optional[Rotation2d] = None) -> Pose2d:
        \"\"\"Convert to a Pose2d, optionally with a rotation (defaults to 0 deg).\"\"\"
        return Pose2d(
            translation=Translation2d(x=self.x, y=self.y),
            rotation=rotation if rotation is not None else Rotation2d(),
        )"""));
  }

  // =========================================================================
  // Pose2d
  // =========================================================================

  private static void registerPose2dMethods() {
    PythonGenerator.registerExtraMethods(
        Pose2d.class,
        m(
            """
    def plus(self, other: Transform2d) -> Pose2d:
        \"\"\"Apply a transform (equivalent to transform_by).\"\"\"
        return self.transform_by(other)"""),
        m(
            """
    def transform_by(self, transform: Transform2d) -> Pose2d:
        \"\"\"Apply a Transform2d to this pose (WPILib transformBy).\"\"\"
        t_trans = transform.translation if transform.translation else Translation2d()
        t_rot = transform.rotation if transform.rotation else Rotation2d()
        new_translation = self.translation.plus(t_trans.rotate_by(self.rotation))
        new_rotation = self.rotation.plus(t_rot)
        return Pose2d(translation=new_translation, rotation=new_rotation)"""),
        m(
            """
    def translate_by(self, translation: Translation2d) -> Pose2d:
        \"\"\"Translate this pose by a Translation2d (no rotation change).\"\"\"
        return Pose2d(
            translation=self.translation.plus(translation),
            rotation=Rotation2d(degrees=self.rotation.degrees),
        )"""),
        m(
            """
    def rotate_by(self, rotation: Rotation2d) -> Pose2d:
        \"\"\"Rotate this pose by a Rotation2d.\"\"\"
        return Pose2d(
            translation=self.translation.rotate_by(rotation),
            rotation=self.rotation.plus(rotation),
        )"""),
        m(
            """
    def rotate_around(self, point: Translation2d, rotation: Rotation2d) -> Pose2d:
        \"\"\"Rotate this pose around a given point.\"\"\"
        new_translation = self.translation.minus(point).rotate_by(rotation).plus(point)
        new_rotation = self.rotation.plus(rotation)
        return Pose2d(translation=new_translation, rotation=new_rotation)"""),
        m(
            """
    def relative_to(self, other: Pose2d) -> Pose2d:
        \"\"\"Express this pose relative to other's coordinate frame.\"\"\"
        inv_rot = other.rotation.unary_minus()
        delta = self.translation.minus(other.translation).rotate_by(inv_rot)
        new_rot = self.rotation.minus(other.rotation)
        return Pose2d(translation=delta, rotation=new_rot)"""),
        m(
            """
    def inverse(self) -> Pose2d:
        inv_rot = self.rotation.unary_minus()
        inv_trans = self.translation.unary_minus().rotate_by(inv_rot)
        return Pose2d(translation=inv_trans, rotation=inv_rot)"""),
        // conversions
        m(
            """
    def to_pose3d(self, z: float = 0.0) -> Pose3d:
        return Pose3d(
            translation=Translation3d(x=self.translation.x, y=self.translation.y, z=z),
            rotation=Rotation3d(yaw=self.rotation.degrees),
        )"""),
        m(
            """
    def to_translation2d(self) -> Translation2d:
        return Translation2d(x=self.translation.x, y=self.translation.y)"""));
  }

  // =========================================================================
  // Transform2d
  // =========================================================================

  private static void registerTransform2dMethods() {
    PythonGenerator.registerExtraMethods(
        Transform2d.class,
        m(
            """
    def plus(self, other: Transform2d) -> Transform2d:
        \"\"\"Compose two transforms.\"\"\"
        return Transform2d(
            translation=self.translation.plus(other.translation.rotate_by(self.rotation)),
            rotation=self.rotation.plus(other.rotation),
        )"""),
        m(
            """
    def inverse(self) -> Transform2d:
        inv_rot = self.rotation.unary_minus()
        inv_trans = self.translation.unary_minus().rotate_by(inv_rot)
        return Transform2d(translation=inv_trans, rotation=inv_rot)"""));
  }

  // =========================================================================
  // Rotation3d
  // =========================================================================

  private static void registerRotation3dMethods() {
    PythonGenerator.registerExtraMethods(
        Rotation3d.class,
        m(
            """
    def plus(self, other: Rotation3d) -> Rotation3d:
        return Rotation3d(roll=self.roll + other.roll, pitch=self.pitch + other.pitch, yaw=self.yaw + other.yaw)"""),
        m(
            """
    def minus(self, other: Rotation3d) -> Rotation3d:
        return Rotation3d(roll=self.roll - other.roll, pitch=self.pitch - other.pitch, yaw=self.yaw - other.yaw)"""),
        m(
            """
    def unary_minus(self) -> Rotation3d:
        return Rotation3d(roll=-self.roll, pitch=-self.pitch, yaw=-self.yaw)"""),
        m(
            """
    def to_rotation2d(self) -> Rotation2d:
        return Rotation2d(degrees=self.yaw)"""));
  }

  // =========================================================================
  // Translation3d
  // =========================================================================

  private static void registerTranslation3dMethods() {
    PythonGenerator.registerExtraMethods(
        Translation3d.class,
        m(
            """
    def plus(self, other: Translation3d) -> Translation3d:
        return Translation3d(x=self.x + other.x, y=self.y + other.y, z=self.z + other.z)"""),
        m(
            """
    def minus(self, other: Translation3d) -> Translation3d:
        return Translation3d(x=self.x - other.x, y=self.y - other.y, z=self.z - other.z)"""),
        m(
            """
    def unary_minus(self) -> Translation3d:
        return Translation3d(x=-self.x, y=-self.y, z=-self.z)"""),
        m(
            """
    def times(self, scalar: float) -> Translation3d:
        return Translation3d(x=self.x * scalar, y=self.y * scalar, z=self.z * scalar)"""),
        m(
            """
    @property
    def norm(self) -> float:
        return _math.sqrt(self.x ** 2 + self.y ** 2 + self.z ** 2)"""),
        m(
            """
    def distance(self, other: Translation3d) -> float:
        return self.minus(other).norm"""),
        // conversions
        m(
            """
    def to_translation2d(self) -> Translation2d:
        return Translation2d(x=self.x, y=self.y)"""),
        m(
            """
    def to_pose2d(self, rotation: Optional[Rotation2d] = None) -> Pose2d:
        \"\"\"Convert to a Pose2d (dropping Z), optionally with a rotation.\"\"\"
        return Pose2d(
            translation=Translation2d(x=self.x, y=self.y),
            rotation=rotation if rotation is not None else Rotation2d(),
        )"""));
  }

  // =========================================================================
  // Pose3d
  // =========================================================================

  private static void registerPose3dMethods() {
    PythonGenerator.registerExtraMethods(
        Pose3d.class,
        m(
            """
    def translate_by(self, translation: Translation3d) -> Pose3d:
        return Pose3d(
            translation=self.translation.plus(translation),
            rotation=Rotation3d(roll=self.rotation.roll, pitch=self.rotation.pitch, yaw=self.rotation.yaw),
        )"""),
        // conversions
        m(
            """
    def to_pose2d(self) -> Pose2d:
        return Pose2d(
            translation=Translation2d(x=self.translation.x, y=self.translation.y),
            rotation=Rotation2d(degrees=self.rotation.yaw),
        )"""),
        m(
            """
    def to_translation2d(self) -> Translation2d:
        return Translation2d(x=self.translation.x, y=self.translation.y)"""));
  }

  // =========================================================================
  // Transform3d
  // =========================================================================

  private static void registerTransform3dMethods() {
    PythonGenerator.registerExtraMethods(
        Transform3d.class,
        m(
            """
    def plus(self, other: Transform3d) -> Transform3d:
        return Transform3d(
            translation=self.translation.plus(other.translation),
            rotation=self.rotation.plus(other.rotation),
        )"""),
        m(
            """
    def inverse(self) -> Transform3d:
        return Transform3d(
            translation=self.translation.unary_minus(),
            rotation=self.rotation.unary_minus(),
        )"""));
  }

  /**
   * Helper to trim text block leading whitespace while keeping relative indentation. Text blocks
   * may have leading newlines which we strip.
   */
  private static String m(String textBlock) {
    // Strip the leading newline that text blocks produce
    if (textBlock.startsWith("\n")) {
      textBlock = textBlock.substring(1);
    }
    return textBlock;
  }
}
