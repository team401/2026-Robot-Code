package frc.robot.autogen;

import coppercore.parameter_tools.json.annotations.JsonSubtype;
import coppercore.parameter_tools.json.annotations.JsonType;
import frc.robot.auto.Auto;
import frc.robot.auto.AutoAction;
import frc.robot.auto.Autos;
import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.Map;

/**
 * Populates the {@code type} discriminator field on every {@link AutoAction} in an object graph.
 *
 * <p>coppercore's polymorphic adapter reads the discriminator on deserialize but does NOT write it
 * on serialize — the value comes from the plain {@code AutoAction.type} field. When authoring autos
 * in Java we never set that field by hand; instead we look each instance up in {@link AutoAction}'s
 * own {@link JsonType} subtype table (the single source of truth) and set {@code type} from the
 * runtime class.
 */
public final class TypeTagger {
  private TypeTagger() {}

  private static final Map<Class<?>, String> NAME_BY_CLASS = buildNameTable();

  private static Map<Class<?>, String> buildNameTable() {
    Map<Class<?>, String> table = new HashMap<>();
    JsonType jsonType = AutoAction.class.getAnnotation(JsonType.class);
    if (jsonType == null) {
      throw new IllegalStateException("AutoAction is missing its @JsonType annotation");
    }
    for (JsonSubtype subtype : jsonType.subtypes()) {
      table.put(subtype.clazz(), subtype.name());
    }
    return table;
  }

  /** Recursively tags every AutoAction reachable from the given Autos object. */
  public static void tag(Autos autos) {
    autos.autos.values().forEach(TypeTagger::tagAuto);
    autos.routines.values().forEach(TypeTagger::tagAction);
  }

  private static void tagAuto(Auto auto) {
    tagAction(readField(auto, Auto.class, "rootAction"));
  }

  private static void tagAction(Object value) {
    if (value == null) {
      return;
    }
    if (value instanceof AutoAction action) {
      String name = NAME_BY_CLASS.get(action.getClass());
      if (name == null) {
        throw new IllegalStateException(
            "No @JsonSubtype registered for " + action.getClass().getName());
      }
      action.type = name;
      // Recurse into any AutoAction-typed fields (e.g. Sequence.actions, Deadline.deadline).
      for (Field field : allFields(action.getClass())) {
        recurse(get(field, action));
      }
    }
  }

  private static void recurse(Object value) {
    if (value instanceof AutoAction) {
      tagAction(value);
    } else if (value instanceof Object[] array) {
      for (Object element : array) {
        recurse(element);
      }
    } else if (value instanceof Iterable<?> iterable) {
      for (Object element : iterable) {
        recurse(element);
      }
    }
  }

  private static Iterable<Field> allFields(Class<?> clazz) {
    java.util.List<Field> fields = new java.util.ArrayList<>();
    for (Class<?> c = clazz; c != null && c != Object.class; c = c.getSuperclass()) {
      for (Field field : c.getDeclaredFields()) {
        fields.add(field);
      }
    }
    return fields;
  }

  private static Object readField(Object target, Class<?> declaringClass, String name) {
    try {
      Field field = declaringClass.getDeclaredField(name);
      field.setAccessible(true);
      return field.get(target);
    } catch (ReflectiveOperationException e) {
      throw new RuntimeException(e);
    }
  }

  private static Object get(Field field, Object target) {
    try {
      field.setAccessible(true);
      return field.get(target);
    } catch (IllegalAccessException e) {
      return null;
    }
  }
}
