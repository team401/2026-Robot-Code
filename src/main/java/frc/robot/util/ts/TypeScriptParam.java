package frc.robot.util.ts;

import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;

/**
 * Defines a single parameter for a {@link TypeScriptMethod}.
 *
 * <p>Example: {@code @TypeScriptParam(name = "target", type = "APTarget")}
 */
@Retention(RetentionPolicy.RUNTIME)
public @interface TypeScriptParam {
  /** The parameter name. */
  String name();

  /** The TypeScript type of the parameter. */
  String type();

  /** Whether the parameter is optional (appends {@code ?} to the name). Defaults to false. */
  boolean optional() default false;
}
