package frc.robot.util.ts;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/**
 * Marks a field as optional in the generated Python dataclass. The field will be emitted as {@code
 * Optional[T] = None}.
 */
@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.FIELD)
public @interface GeneratedOptional {}
