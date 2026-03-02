package frc.robot.util.ts;

import java.lang.annotation.ElementType;
import java.lang.annotation.Repeatable;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

// Written with help of Copilot (Claude model)

/**
 * Injects raw TypeScript code into the generated file, appended after the class definition. Useful
 * for standalone utility functions, module-level exports, or additional imports.
 *
 * <p>Use {@code atTop = true} to place the code at the top of the file (e.g., for imports) instead
 * of after the class.
 *
 * <p>Example:
 *
 * <pre>
 * &#64;TypeScriptAppend("import * as AutoLib from './AutoLib.js';")
 * &#64;TypeScriptAppend(value = "export function doSomething() { ... }", atTop = false)
 * public class MyAction extends AutoAction { ... }
 * </pre>
 */
@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.TYPE)
@Repeatable(TypeScriptAppends.class)
public @interface TypeScriptAppend {
  /** The raw TypeScript code to inject. */
  String value();

  /** If true, the code is placed at the top of the file (before any classes). Defaults to false. */
  boolean atTop() default false;
}
