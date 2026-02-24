package frc.robot.util.ts;

import java.lang.annotation.ElementType;
import java.lang.annotation.Repeatable;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/**
 * Adds a TypeScript import statement to the top of the generated file. Place on a Java class that
 * is processed by TypeScriptGenerator.
 *
 * <p>Supports two import styles:
 *
 * <ul>
 *   <li>Namespace import: {@code import * as Name from 'module';}
 *       <pre>&#64;TypeScriptImport(module = "./AutoLib.js", alias = "AutoLib")</pre>
 *   <li>Named import: {@code import { a, b } from 'module';}
 *       <pre>&#64;TypeScriptImport(module = "./AutoLib.js", members = {"addCommand", "add"})</pre>
 * </ul>
 *
 * <p>If {@code alias} is set, a namespace import is generated. If {@code members} is set, a named
 * import is generated. Do not set both.
 */
@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.TYPE)
@Repeatable(TypeScriptImports.class)
public @interface TypeScriptImport {
  /** The module path (e.g., {@code "./AutoLib.js"} or {@code "@/typescript/AutoAction.js"}). */
  String module();

  /**
   * If set, generates a namespace import: {@code import * as <alias> from '<module>';}. Mutually
   * exclusive with {@link #members}.
   */
  String alias() default "";

  /**
   * If set, generates a named import: {@code import { <members> } from '<module>';}. Mutually
   * exclusive with {@link #alias}.
   */
  String[] members() default {};
}
