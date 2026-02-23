package frc.robot.util.ts;

import com.google.gson.FieldNamingStrategy;
import coppercore.parameter_tools.json.JSONSyncConfig;
import coppercore.parameter_tools.json.JSONSyncConfigBuilder;
import coppercore.parameter_tools.json.annotations.JSONExclude;
import coppercore.parameter_tools.json.annotations.JsonSubtype;
import coppercore.parameter_tools.json.annotations.JsonType;
import coppercore.parameter_tools.json.helpers.JSONConverter;
import coppercore.parameter_tools.json.helpers.JSONConverter.ConversionException;
import coppercore.parameter_tools.json.helpers.JSONObject;
import coppercore.parameter_tools.json.strategies.JSONNamingStrategy;
import coppercore.parameter_tools.json.strategies.JSONPrimitiveCheckStrategy;
import java.io.FileWriter;
import java.io.IOException;
import java.lang.reflect.*;
import java.util.*;

/**
 * Generates TypeScript definitions from any Java class using the same config rules as JSONSync /
 * JSONHandler.
 */
public final class TypeScriptGenerator {

  private static final JSONSyncConfig defaultConfig = new JSONSyncConfigBuilder().build();

  private static FieldNamingStrategy namingStrategy;

  private static final Map<Class<?>, String> primitiveMap = new HashMap<>();
  private static final Set<Class<?>> generated = new HashSet<>();
  private static final StringBuilder output = new StringBuilder();

  private static final boolean shouldSkipField(Field field) {
    return Modifier.isStatic(field.getModifiers())
        || Modifier.isTransient(field.getModifiers())
        || field.isAnnotationPresent(JSONExclude.class);
  }

  static {
    primitiveMap.put(int.class, "number");
    primitiveMap.put(Integer.class, "number");
    primitiveMap.put(double.class, "number");
    primitiveMap.put(Double.class, "number");
    primitiveMap.put(float.class, "number");
    primitiveMap.put(Float.class, "number");
    primitiveMap.put(long.class, "number");
    primitiveMap.put(Long.class, "number");
    primitiveMap.put(short.class, "number");
    primitiveMap.put(Short.class, "number");
    primitiveMap.put(byte.class, "number");
    primitiveMap.put(Byte.class, "number");

    primitiveMap.put(boolean.class, "boolean");
    primitiveMap.put(Boolean.class, "boolean");

    primitiveMap.put(String.class, "string");
    primitiveMap.put(char.class, "string");
    primitiveMap.put(Character.class, "string");
  }

  // ============================================
  // Public Entry
  // ============================================

  public static void generateFor(Object root, String filePath) {
    generated.clear();
    output.setLength(0);

    namingStrategy =
        JSONPrimitiveCheckStrategy.checkForPrimitives(
            new JSONNamingStrategy(defaultConfig.namingPolicy(), defaultConfig), defaultConfig);

    generateType(root.getClass());

    try (FileWriter writer = new FileWriter(filePath)) {
      writer.write(output.toString());
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }

  // ============================================
  // Core Generation
  // ============================================

  private static void generateType(Class<?> clazz) {

    if (generated.contains(clazz)) return;
    if (isPrimitive(clazz)) return;
    if (clazz.isEnum()) {
      generateEnum(clazz);
      return;
    }

    generated.add(clazz);

    // Handle polymorphic types
    if (clazz.isAnnotationPresent(JsonType.class)) {
      generateDiscriminatedUnion(clazz);
      return;
    }

    generateInterface(clazz);
  }

  // ============================================
  // Interfaces
  // ============================================

  public static Set<Class<?>> emitted = new HashSet<>();

  public static void generateInterface(Class<?> clazz) {
    if (emitted.contains(clazz)) return;
    emitted.add(clazz);

    StringBuilder sb = new StringBuilder();
    sb.append("export interface ").append(clazz.getSimpleName()).append(" {\n");

    for (Field field : clazz.getDeclaredFields()) {
      if (shouldSkipField(field)) continue;

      Class<?> fieldType = field.getType();

      // Use JSONConverter if available
      try {
        fieldType = JSONConverter.convert(fieldType);
      } catch (ConversionException ignored) {
      }

      String tsType = resolveType(field.getGenericType());

      sb.append("  ").append(field.getName()).append(": ").append(tsType).append(";\n");
    }

    sb.append("}\n\n");
    output.append(sb);
  }

  // ============================================
  // Polymorphism
  // ============================================

  private static void generateDiscriminatedUnion(Class<?> baseClass) {

    JsonType typeAnnotation = baseClass.getAnnotation(JsonType.class);
    String discriminator = typeAnnotation.property();

    List<String> subtypeNames = new ArrayList<>();

    for (JsonSubtype subtype : typeAnnotation.subtypes()) {
      Class<?> subClass = subtype.clazz();
      String name = subtype.name();

      subtypeNames.add(subClass.getSimpleName());

      generateSubtype(subClass, discriminator, name);
    }

    output
        .append("export type ")
        .append(baseClass.getSimpleName())
        .append(" = ")
        .append(String.join(" | ", subtypeNames))
        .append(";\n\n");
  }

  private static void generateSubtype(
      Class<?> clazz, String discriminator, String discriminatorValue) {

    if (generated.contains(clazz)) return;
    generated.add(clazz);

    output.append("export interface ").append(clazz.getSimpleName()).append(" {\n");

    output
        .append("  ")
        .append(discriminator)
        .append(": \"")
        .append(discriminatorValue)
        .append("\";\n");

    for (Field field : getAllFields(clazz)) {

      if (shouldSkipField(field)) continue;
      if (field.getName().equals(discriminator)) continue;

      String fieldName = namingStrategy.translateName(field);
      String tsType = resolveType(field.getGenericType());

      output.append("  ").append(fieldName).append(": ").append(tsType).append(";\n");
    }

    output.append("}\n\n");
  }

  // ============================================
  // Type Resolution
  // ============================================

  private static String resolveType(Type type) {
    if (type instanceof Class<?> clazz) {

      // Check if class has a JSON wrapper
      try {
        Class<? extends JSONObject<?>> wrapper = JSONConverter.convert(clazz);
        if (wrapper != null) {
          clazz = wrapper; // Use the wrapper for generating TypeScript
        }
      } catch (JSONConverter.ConversionException ignored) {
        // No wrapper, continue as normal
      }

      if (primitiveMap.containsKey(clazz)) {
        return primitiveMap.get(clazz);
      }

      if (clazz.isArray()) {
        return resolveType(clazz.getComponentType()) + "[]";
      }

      if (Map.class.isAssignableFrom(clazz)) {
        return "{ [key: string]: any }";
      }

      generateType(clazz);
      return clazz.getSimpleName();
    }

    if (type instanceof ParameterizedType pt) {
      Type raw = pt.getRawType();
      Type[] args = pt.getActualTypeArguments();
      if (raw instanceof Class<?> rawClass) {

        // List<T>
        if (Collection.class.isAssignableFrom(rawClass)) {
          return resolveType(args[0]) + "[]";
        }

        // Map<String, T>
        if (Map.class.isAssignableFrom(rawClass)) {
          return "{ [key: string]: " + resolveType(args[1]) + " }";
        }

        // Check wrapper for raw class
        try {
          Class<? extends JSONObject<?>> wrapper = JSONConverter.convert(rawClass);
          if (wrapper != null) {
            rawClass = wrapper;
          }
        } catch (JSONConverter.ConversionException ignored) {
        }

        generateType(rawClass);
        return rawClass.getSimpleName();
      }
    }

    return "any";
  }

  // ============================================
  // Enums
  // ============================================

  private static void generateEnum(Class<?> clazz) {

    if (generated.contains(clazz)) return;
    generated.add(clazz);

    output.append("export type ").append(clazz.getSimpleName()).append(" = ");

    Object[] constants = clazz.getEnumConstants();
    List<String> values = new ArrayList<>();

    for (Object constant : constants) {
      values.add("\"" + constant.toString() + "\"");
    }

    output.append(String.join(" | ", values)).append(";\n\n");
  }

  // ============================================
  // Utilities
  // ============================================

  private static boolean isPrimitive(Class<?> clazz) {
    return clazz.isPrimitive() || primitiveMap.containsKey(clazz);
  }

  private static List<Field> getAllFields(Class<?> clazz) {
    List<Field> fields = new ArrayList<>();
    while (clazz != null) {
      fields.addAll(Arrays.asList(clazz.getDeclaredFields()));
      clazz = clazz.getSuperclass();
    }
    return fields;
  }
}
