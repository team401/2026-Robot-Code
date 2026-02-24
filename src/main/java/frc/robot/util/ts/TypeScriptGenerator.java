package frc.robot.util.ts;

import com.google.gson.FieldNamingStrategy;
import coppercore.parameter_tools.json.JSONSyncConfig;
import coppercore.parameter_tools.json.JSONSyncConfigBuilder;
import coppercore.parameter_tools.json.adapters.measure.JSONMeasure;
import coppercore.parameter_tools.json.annotations.JSONExclude;
import coppercore.parameter_tools.json.annotations.JsonSubtype;
import coppercore.parameter_tools.json.annotations.JsonType;
import coppercore.parameter_tools.json.helpers.JSONConverter;
import coppercore.parameter_tools.json.helpers.JSONConverter.ConversionException;
import coppercore.parameter_tools.json.helpers.JSONObject;
import coppercore.parameter_tools.json.strategies.JSONNamingStrategy;
import coppercore.parameter_tools.json.strategies.JSONPrimitiveCheckStrategy;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.FileWriter;
import java.io.IOException;
import java.lang.reflect.*;
import java.nio.file.Path;
import java.util.*;

// Written with the help of ChatGPT.

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
  private static boolean hasGeneratedUnits = false;
  private static final String BASE_FILE_PATH = "typescript";
  private static final Path BASE_PATH = Filesystem.getDeployDirectory().toPath().resolve(BASE_FILE_PATH);
  private static String unitFilePath = "Units.ts";
  private static boolean hasImportedUnits = false;

  public static Set<Class<?>> emitted = new HashSet<>();

  private static final boolean shouldSkipField(Field field) {
    return Modifier.isStatic(field.getModifiers())
        || Modifier.isTransient(field.getModifiers())
        || field.isAnnotationPresent(JSONExclude.class);
  }

  public static void setUnitFilePath(String path) {
    unitFilePath = path;
  }

  public static void generateUnitsFile() {
    if (hasGeneratedUnits) return;
    hasGeneratedUnits = true;
    StringBuilder sb = new StringBuilder();
    sb.append("export type Measure = {\n");
    sb.append("  value: number;\n");
    sb.append("  unit: string;\n");
    sb.append("};\n\n");

    sb.append("export class Unit<M extends Measure>{\n");
    sb.append("\tconstructor(public name: string) {}\n");
    sb.append("\tof(value: number): M {\n");
    sb.append("\t\treturn { value, unit: this.name } as M;\n");
    sb.append("\t}\n");
    sb.append("}\n");

    Set<Unit> units = new HashSet<>();
    HashMap<Unit, String> baseUnits = new HashMap<>();
    JSONMeasure.unitMap.forEach(
        (name, unitFunc) -> {
          var measure = unitFunc.apply(1.0);
          var unit = measure.unit();
          var baseUnit = unit.getBaseUnit();
          if (baseUnit == null) return;
          if (!baseUnits.containsKey(baseUnit)) {
            var measureName = baseUnit.getClass().getSimpleName();
            if (measureName.equals("PerUnit")) return;
            // remove Unit suffix if it exists
            if (measureName.endsWith("Unit")) {
              measureName = measureName.substring(0, measureName.length() - 4);
            }
            sb.append("export type ").append(measureName).append(" = Measure;\n");
            baseUnits.put(baseUnit, measureName);
          }
          if (units.contains(unit)) return;
          units.add(unit);
          var unitName = unit.name().replace(" ", "_").replace("-", "_");
          if (unitName.equals("<?>")) unitName = "Unitless";
          sb.append("export const ")
              .append(unitName)
              .append(" = new Unit<")
              .append(baseUnits.get(baseUnit))
              .append(">(\"")
              .append(unit.name())
              .append("\");\n");
        });

    System.out.println("Generated TypeScript definitions for " + units.size() + " units.");

    try (FileWriter writer =
        new FileWriter(
            BASE_PATH
                .resolve(unitFilePath)
                .toString())) {
      writer.write(sb.toString());
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
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

  private static void initGenerator() {
    generated.clear();
    emitted.clear();
    output.setLength(0);
    hasImportedUnits = false;

    namingStrategy =
        JSONPrimitiveCheckStrategy.checkForPrimitives(
            new JSONNamingStrategy(defaultConfig.namingPolicy(), defaultConfig), defaultConfig);
  }

  private static void writeOutput(String filePath) {
    try (FileWriter writer = new FileWriter(BASE_PATH.resolve(filePath).toString())) {
      writer.write(output.toString());
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }

  public static void generateForObjects(String filePath, Object... roots) {
    initGenerator();

    for (Object root : roots) {
      generateType(root.getClass());
    }

    writeOutput(filePath);
  }

  public static void generateForClasses(String filePath, Class<?>... rootClasses) {
    initGenerator();

    for (Class<?> rootClass : rootClasses) {
      generateType(rootClass);
    }

    writeOutput(filePath);
  }

  // ============================================
  // Core Generation
  // ============================================

  private static void generateType(Class<?> clazz) {
    generateType(clazz, clazz.getSimpleName());
  }

  private static void generateType(Class<?> clazz, String suggestedName) {

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

    generateInterface(clazz, suggestedName);
  }

  // ============================================
  // Interfaces
  // ============================================

  public static void generateInterface(Class<?> clazz, String suggestedName) {
    suggestedName = suggestedName != null ? suggestedName : clazz.getSimpleName();
    if (emitted.contains(clazz)) return;
    emitted.add(clazz);

    StringBuilder sb = new StringBuilder();
    sb.append("export interface ").append(suggestedName).append(" {\n");

    for (Field field : clazz.getDeclaredFields()) {
      if (shouldSkipField(field)) continue;

      Class<?> fieldType = field.getType();
      Class<?> originalFieldType = fieldType;

      // Use JSONConverter if available
      try {
        fieldType = JSONConverter.convert(fieldType);
      } catch (ConversionException ignored) {
      }

      String tsType = resolveType(field.getGenericType(), originalFieldType.getSimpleName());

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

    StringBuilder sb = new StringBuilder();

    sb.append("export interface ").append(clazz.getSimpleName()).append(" {\n");

    sb.append("  ").append(discriminator).append(": \"").append(discriminatorValue).append("\";\n");

    for (Field field : getAllFields(clazz)) {

      if (shouldSkipField(field)) continue;
      if (field.getName().equals(discriminator)) continue;

      String fieldName = namingStrategy.translateName(field);
      String tsType = resolveType(field.getGenericType());

      sb.append("  ").append(fieldName).append(": ").append(tsType).append(";\n");
    }

    sb.append("}\n\n");
    output.append(sb);
  }

  // ============================================
  // Type Resolution
  // ============================================

  private static String resolveType(Type type) {
    return resolveType(type, null);
  }

  private static String resolveType(Type type, String suggestedName) {
    if (type instanceof Class<?> clazz) {

      boolean isUnit = isUnitType(clazz);
      if (isUnit) {
        ensureUnitsIncluded();
      }

      // Check if class has a JSON wrapper
      try {
        Class<? extends JSONObject<?>> wrapper = JSONConverter.convert(clazz);
        if (wrapper != null) {
          suggestedName = clazz.getSimpleName();
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
      suggestedName = suggestedName != null ? suggestedName : clazz.getSimpleName();
      if (isUnit) {
        // For units, we want to return the Measure type instead of the wrapper interface
        // Maybe still call generateType in case of Per Units
        return "Units." + suggestedName;
      }
      generateType(clazz, suggestedName);
      return suggestedName;
    }

    if (type instanceof ParameterizedType pt) {
      Type raw = pt.getRawType();
      Type[] args = pt.getActualTypeArguments();
      if (raw instanceof Class<?> rawClass) {

        boolean isUnit = isUnitType(rawClass);

        if (isUnit) {
          ensureUnitsIncluded();
        }

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
            suggestedName = rawClass.getSimpleName();
            rawClass = wrapper;
          }
        } catch (JSONConverter.ConversionException ignored) {
        }

        if (isUnitType(rawClass)) {
          isUnit = true;
          ensureUnitsIncluded();
        }

        suggestedName = suggestedName != null ? suggestedName : rawClass.getSimpleName();
        generateType(rawClass, suggestedName);
        if (isUnit) {
          // For units, we want to return the Measure type instead of the wrapper interface
          return "Units." + suggestedName;
        }
        return suggestedName;
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

  private static boolean isUnitType(Class<?> clazz) {
    return Measure.class.isAssignableFrom(clazz) || Unit.class.isAssignableFrom(clazz);
  }

  private static void ensureUnitsFile() {
    if (hasGeneratedUnits) return;
    generateUnitsFile();
  }

  private static void ensureUnitsIncluded() {
    ensureUnitsFile();
    if (hasImportedUnits) return;
    hasImportedUnits = true;
    // TODO: add import statement to the output for the units file if it hasn't been added already
    output
        .append("import * as Units from './")
        .append(unitFilePath.replace(".ts", ".js"))
        .append("';\n\n");
  }
}
