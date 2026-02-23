package frc.robot.util.ts;

import coppercore.parameter_tools.json.annotations.JSONExclude;
import coppercore.parameter_tools.json.helpers.JSONConverter;
import java.io.FileWriter;
import java.io.IOException;
import java.lang.reflect.*;
import java.util.*;

public class TypescriptConverter {

  // Map of Java types to TypeScript types
  public static final Map<Class<?>, String> typeMap = new HashMap<>();

  public static FileWriter writer;

  /* ============================================================
   *  File Writer
   * ============================================================ */

  public static void initializeFileWriter(String filePath) {
    try {
      writer = new FileWriter(filePath);
    } catch (IOException e) {
      e.printStackTrace();
    }
  }

  public static void closeFileWriter() {
    try {
      if (writer != null) {
        writer.close();
      }
    } catch (IOException e) {
      e.printStackTrace();
    }
  }

  /* ============================================================
   *  Base Type Mapping
   * ============================================================ */

  public static void addBaseTypes() {

    // primitives
    typeMap.put(int.class, "number");
    typeMap.put(double.class, "number");
    typeMap.put(float.class, "number");
    typeMap.put(long.class, "number");
    typeMap.put(short.class, "number");
    typeMap.put(byte.class, "number");
    typeMap.put(boolean.class, "boolean");
    typeMap.put(char.class, "string");
    typeMap.put(void.class, "void");

    // boxed
    typeMap.put(Integer.class, "number");
    typeMap.put(Double.class, "number");
    typeMap.put(Float.class, "number");
    typeMap.put(Long.class, "number");
    typeMap.put(Short.class, "number");
    typeMap.put(Byte.class, "number");
    typeMap.put(Boolean.class, "boolean");
    typeMap.put(Character.class, "string");

    typeMap.put(String.class, "string");
    typeMap.put(Object.class, "any");
  }

  /* ============================================================
   *  Type Resolution (Recursive + Generic Safe)
   * ============================================================ */

  public static String getTypeScriptType(Type javaType) {

    // ----------------------------
    // Raw Class
    // ----------------------------
    if (javaType instanceof Class<?> clazz) {

      // Array
      if (clazz.isArray()) {
        return getTypeScriptType(clazz.getComponentType()) + "[]";
      }

      // Enum -> string union
      if (clazz.isEnum()) {
        Object[] constants = clazz.getEnumConstants();
        return Arrays.stream(constants)
            .map(Object::toString)
            .map(s -> "\"" + s + "\"")
            .reduce((a, b) -> a + " | " + b)
            .orElse("string");
      }

      return typeMap.getOrDefault(clazz, clazz.getSimpleName());
    }

    // ----------------------------
    // Parameterized Type
    // ----------------------------
    if (javaType instanceof ParameterizedType parameterizedType) {

      Type rawType = parameterizedType.getRawType();
      Type[] typeArgs = parameterizedType.getActualTypeArguments();

      if (rawType instanceof Class<?> rawClass) {

        // List<T> -> T[]
        if (List.class.isAssignableFrom(rawClass)) {
          return getTypeScriptType(typeArgs[0]) + "[]";
        }

        // Set<T> -> T[]
        if (Set.class.isAssignableFrom(rawClass)) {
          return getTypeScriptType(typeArgs[0]) + "[]";
        }

        // Map<K, V> -> { [key: K]: V }
        if (Map.class.isAssignableFrom(rawClass)) {
          return "{ [key: "
              + getTypeScriptType(typeArgs[0])
              + "]: "
              + getTypeScriptType(typeArgs[1])
              + " }";
        }

        // Optional<T> -> T | null
        if (Optional.class.isAssignableFrom(rawClass)) {
          return getTypeScriptType(typeArgs[0]) + " | null";
        }

        // Generic custom class Foo<T>
        StringBuilder sb = new StringBuilder();
        sb.append(typeMap.getOrDefault(rawClass, rawClass.getSimpleName()));
        sb.append("<");

        for (int i = 0; i < typeArgs.length; i++) {
          sb.append(getTypeScriptType(typeArgs[i]));
          if (i < typeArgs.length - 1) {
            sb.append(", ");
          }
        }

        sb.append(">");
        return sb.toString();
      }
    }

    // ----------------------------
    // Type Variables (T)
    // ----------------------------
    if (javaType instanceof TypeVariable<?>) {
      return "any";
    }

    // ----------------------------
    // Wildcards (?)
    // ----------------------------
    if (javaType instanceof WildcardType wildcardType) {
      Type[] upperBounds = wildcardType.getUpperBounds();
      if (upperBounds.length > 0) {
        return getTypeScriptType(upperBounds[0]);
      }
      return "any";
    }

    return "any";
  }

  /* ============================================================
   *  TypeScript Class Representation
   * ============================================================ */

  public record TypeScriptField(String name, Type type) {}

  public static class TypeScriptClass {

    public final String name;
    public final Class<?> javaClass;
    public final List<TypeScriptField> fields = new ArrayList<>();

    public TypeScriptClass(String name, Class<?> javaClass) {
      this.name = name;
      this.javaClass = javaClass;
    }

    public void addField(String fieldName, Type fieldType) {
      fields.add(new TypeScriptField(fieldName, fieldType));
    }

    public String toTypeScript() {
      StringBuilder sb = new StringBuilder();

      sb.append("export interface ").append(name).append(" {\n");

      for (TypeScriptField field : fields) {
        sb.append("  ")
            .append(field.name())
            .append(": ")
            .append(getTypeScriptType(field.type()))
            .append(";\n");
      }

      sb.append("}\n\n");
      return sb.toString();
    }
  }

  /* ============================================================
   *  Reflection Field Extraction
   * ============================================================ */

  public static void addFieldsToTypeScriptClass(TypeScriptClass tsClass, Class<?> javaClass) {
    for (Field field : javaClass.getDeclaredFields()) {

      if (Modifier.isStatic(field.getModifiers())) continue;
      if (Modifier.isTransient(field.getModifiers())) continue;
      if (field.isAnnotationPresent(JSONExclude.class)) continue;

      tsClass.addField(field.getName(), field.getGenericType());
    }
  }

  /* ============================================================
   *  Writing Classes
   * ============================================================ */

  private static void writeTypeScriptClass(TypeScriptClass tsClass) {
    try {
      writer.write(tsClass.toTypeScript());
    } catch (IOException e) {
      e.printStackTrace();
    }
  }

  public static void addTypeScriptClasses(List<TypeScriptClass> classes) {

    // Register types first
    for (TypeScriptClass tsClass : classes) {
      typeMap.put(tsClass.javaClass, tsClass.name);
    }

    // Then write
    for (TypeScriptClass tsClass : classes) {
      writeTypeScriptClass(tsClass);
    }
  }

  /* ============================================================
   *  JSON Converter Auto Discovery
   * ============================================================ */

  private static void addJSONConverterClasses() {

    List<TypeScriptClass> converterClasses = new ArrayList<>();

    JSONConverter.jsonMap.forEach(
        (javaClass, jsonClass) -> {
          String tsClassName = javaClass.getSimpleName();

          TypeScriptClass tsClass = new TypeScriptClass(tsClassName, javaClass);

          addFieldsToTypeScriptClass(tsClass, jsonClass);

          converterClasses.add(tsClass);
        });

    addTypeScriptClasses(converterClasses);
  }

  /* ============================================================
   *  Entry Point
   * ============================================================ */

  public static void run(String filePath) {

    initializeFileWriter(filePath);
    addBaseTypes();
    addJSONConverterClasses();
    closeFileWriter();
  }
}
