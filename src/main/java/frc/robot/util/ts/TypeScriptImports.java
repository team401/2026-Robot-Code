package frc.robot.util.ts;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/**
 * Container annotation for repeatable {@link TypeScriptImport} annotations. You do not need to use
 * this directly—just apply multiple {@code @TypeScriptImport} annotations to the same class.
 */
@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.TYPE)
public @interface TypeScriptImports {
  TypeScriptImport[] value();
}
