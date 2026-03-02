package frc.robot.util.ts;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

// Written with help of Copilot (Claude model)

/**
 * Container annotation for repeatable {@link TypeScriptMethod} annotations. You do not need to use
 * this directly—just apply multiple {@code @TypeScriptMethod} annotations to the same class.
 */
@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.TYPE)
public @interface TypeScriptMethods {
  TypeScriptMethod[] value();
}
