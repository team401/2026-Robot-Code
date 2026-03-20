package frc.robot.util.ts;

import java.lang.annotation.ElementType;
import java.lang.annotation.Repeatable;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/**
 * Injects raw Python code into the generated file. Place on a Java class that is processed by
 * PythonGenerator.
 *
 * <p>Use {@code atTop = true} to place the code at the top of the file (e.g., for imports) instead
 * of after the class.
 */
@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.TYPE)
@Repeatable(PythonAppends.class)
public @interface PythonAppend {
  /** The raw Python code to inject. */
  String value();

  /** If true, the code is placed at the top of the file (before any classes). Defaults to false. */
  boolean atTop() default false;
}
