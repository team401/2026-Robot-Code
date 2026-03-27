package frc.robot.util.json;

import com.google.gson.JsonDeserializer;
import com.google.gson.JsonSerializer;
import com.google.gson.TypeAdapter;
import com.google.gson.TypeAdapterFactory;
import coppercore.parameter_tools.json.JSONSyncConfig;
import coppercore.parameter_tools.json.JSONSyncConfigBuilder;
import edu.wpi.first.math.Pair;
import java.util.ArrayList;
import java.util.List;

// I forgot to make a method to add typeAdapterFactories.
// And for some reason I made both typeAdapters and typeAdapterFactories private in JSONSyncConfig,
// which means I can't just extend it and add a method to add factories.

public class FixedJSONSyncConfigBuilder extends JSONSyncConfigBuilder {

  /**
   * List of custom type adapters (JsonSerializer, JsonDeserializer, TypeAdapter) to be registered.
   */
  @SuppressWarnings("rawtypes")
  public List<Pair<Class, Object>> typeAdapters = new ArrayList<>();

  /** List of custom TypeAdapterFactory instances to be registered. */
  public List<TypeAdapterFactory> typeAdapterFactories = new ArrayList<>();

  /**
   * Adds a custom JsonDeserializer for a specific class.
   *
   * @param <T> The type of the class.
   * @param clazz The class to associate with the JsonDeserializer.
   * @param adapter The JsonDeserializer to add.
   * @return The builder instance.
   */
  public <T> JSONSyncConfigBuilder addJsonDeserializer(
      Class<T> clazz, JsonDeserializer<T> adapter) {
    typeAdapters.add(new Pair<>(clazz, adapter));
    return this;
  }

  /**
   * Adds a custom JsonSerializer for a specific class.
   *
   * @param <T> The type of the class.
   * @param clazz The class to associate with the JsonSerializer.
   * @param adapter The JsonSerializer to add.
   * @return The builder instance.
   */
  public <T> JSONSyncConfigBuilder addJsonSerializer(Class<T> clazz, JsonSerializer<T> adapter) {
    typeAdapters.add(new Pair<>(clazz, adapter));
    return this;
  }

  /**
   * Adds a custom TypeAdapter for a specific class.
   *
   * @param <T> The type of the class.
   * @param clazz The class to associate with the TypeAdapter.
   * @param adapter The TypeAdapter to add.
   * @return The builder instance.
   */
  public <T> JSONSyncConfigBuilder addJsonTypeAdapter(Class<T> clazz, TypeAdapter<T> adapter) {
    typeAdapters.add(new Pair<>(clazz, adapter));
    return this;
  }

  public <T> JSONSyncConfigBuilder addJsonTypeAdapterFactory(TypeAdapterFactory factory) {
    typeAdapterFactories.add(factory);
    return this;
  }

  /**
   * Builds the configuration object.
   *
   * @return A JSONSyncConfig instance.
   */
  public JSONSyncConfig build() {
    return new JSONSyncConfig(
        serializeNulls,
        prettyPrinting,
        excludeFieldsWithoutExposeAnnotation,
        namingPolicy,
        longSerializationPolicy,
        primitiveChecking,
        primitiveCheckPrintAlert,
        primitiveCheckCrash,
        typeAdapters,
        typeAdapterFactories);
  }
}
