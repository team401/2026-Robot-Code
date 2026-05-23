package frc.robot.util.json;

import com.google.gson.Gson;
import com.google.gson.JsonElement;
import com.google.gson.JsonParseException;
import com.google.gson.JsonParser;
import com.google.gson.TypeAdapter;
import com.google.gson.TypeAdapterFactory;
import com.google.gson.reflect.TypeToken;
import com.google.gson.stream.JsonReader;
import com.google.gson.stream.JsonWriter;
import coppercore.parameter_tools.json.annotations.JsonSubtype;
import coppercore.parameter_tools.json.annotations.JsonType;
import java.io.IOException;
import java.lang.reflect.Type;
import java.util.Arrays;

/**
 * A custom deserializer for handling polymorphic JSON deserialization. This deserializer uses a
 * {@link JsonType} annotation on the target class to determine the property that specifies the
 * subtype and the mapping of property values to specific subtypes.
 *
 * @param <T> the base type of the object to deserialize
 */
public class FixedPolymorphTypeAdapterFactory implements TypeAdapterFactory {

  @Override
  public <T> TypeAdapter<T> create(Gson gson, TypeToken<T> type) {
    var clazz = type.getRawType();

    var typeAnnotation = clazz.getAnnotation(JsonType.class);

    if (typeAnnotation == null) {
      return null;
    }

    return new TypeAdapter<T>() {
      @Override
      public T read(JsonReader reader) throws IOException {
        try {
          // Parse into JsonElement first
          JsonElement jsonElement = JsonParser.parseReader(reader);

          JsonType jsonType = clazz.getDeclaredAnnotation(JsonType.class);
          if (jsonType == null) {
            throw new JsonParseException("Missing @JsonType annotation on " + clazz.getName());
          }

          String discriminatorField = jsonType.property();
          String discriminatorValue =
              jsonElement.getAsJsonObject().get(discriminatorField).getAsString();

          JsonSubtype[] subtypes = jsonType.subtypes();

          Type targetType =
              Arrays.stream(subtypes)
                  .filter(sub -> sub.name().equals(discriminatorValue))
                  .findFirst()
                  .orElseThrow(
                      () -> new JsonParseException("Unknown subtype: " + discriminatorValue))
                  .clazz();

          return gson.fromJson(jsonElement, targetType);

        } catch (Exception e) {
          throw new JsonParseException("Failed to deserialize json", e);
        }
      }

      @Override
      public void write(JsonWriter out, T value) throws IOException {
        gson.toJson(value, value.getClass(), out);
      }
    };
  }
}
