package frc.robot.util.ts;

import java.lang.annotation.ElementType;
import java.lang.annotation.Repeatable;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/**
 * Defines a TypeScript method to be injected into the generated class. Place on a Java class that
 * is processed by TypeScriptGenerator. Each annotation defines one method.
 *
 * <p>The generator assembles valid TypeScript method syntax from the structured fields—you don't
 * need to worry about braces, indentation, or formatting.
 *
 * <p>Example:
 *
 * <pre>
 * &#64;TypeScriptMethod(
 *     name = "add",
 *     returnType = "this",
 *     body = {"AutoLib.addCommand(this);", "return this;"},
 *     comment = "Adds this command to the current auto and returns itself for chaining."
 * )
 * public class AutoPilotAction extends AutoAction { ... }
 * </pre>
 *
 * Generates:
 *
 * <pre>
 *   /** Adds this command to the current auto and returns itself for chaining. *&#47;
 *   add(): this {
 *     AutoLib.addCommand(this);
 *     return this;
 *   }
 * </pre>
 */
@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.TYPE)
@Repeatable(TypeScriptMethods.class)
public @interface TypeScriptMethod {
  /** The method name. */
  String name();

  /**
   * The method parameters, each defined as a structured {@link TypeScriptParam}.
   *
   * <p>Example:
   *
   * <pre>
   * params = {
   *     &#64;TypeScriptParam(name = "target", type = "APTarget"),
   *     &#64;TypeScriptParam(name = "profile", type = "APProfile", optional = true)
   * }
   * </pre>
   */
  TypeScriptParam[] params() default {};

  /**
   * The TypeScript return type. Use {@code "void"} for no return value, {@code "this"} for fluent
   * chaining, or any TypeScript type name. Defaults to {@code "void"}.
   */
  String returnType() default "void";

  /**
   * The method body as individual statements. Each element becomes one indented line inside the
   * method. Do not include braces—the generator adds them.
   *
   * <p>Example: {@code {"AutoLib.addCommand(this);", "return this;"}}
   */
  String[] body();

  /** An optional JSDoc comment for the method. If non-empty, emitted as a {@code /** ... *\/}. */
  String comment() default "";

  /** Whether this is a static method. Defaults to false. */
  boolean isStatic() default false;
}
