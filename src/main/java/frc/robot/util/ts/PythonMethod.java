package frc.robot.util.ts;

import java.lang.annotation.ElementType;
import java.lang.annotation.Repeatable;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/**
 * Defines a Python method to be injected into the generated dataclass. Place on a Java class that
 * is processed by PythonGenerator.
 *
 * <p>Example:
 *
 * <pre>
 * &#64;PythonMethod(
 *     name = "add",
 *     returnType = "self",
 *     body = {"_call_hook(self)", "return self"},
 *     comment = "Adds this command to the current auto and returns itself for chaining."
 * )
 * </pre>
 */
@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.TYPE)
@Repeatable(PythonMethods.class)
public @interface PythonMethod {
  /** The method name. */
  String name();

  /** The Python return type annotation (e.g. "self", "None"). Defaults to "None". */
  String returnType() default "None";

  /** The method body as individual statements. Each element becomes one indented line. */
  String[] body();

  /** An optional docstring for the method. */
  String comment() default "";

  /** Whether this is a static method. Defaults to false. */
  boolean isStatic() default false;
}
