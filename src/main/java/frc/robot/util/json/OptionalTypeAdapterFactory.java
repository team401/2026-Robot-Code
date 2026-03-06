package frc.robot.util.json;

import com.google.gson.Gson;
import com.google.gson.TypeAdapter;
import com.google.gson.TypeAdapterFactory;
import com.google.gson.reflect.TypeToken;
import com.google.gson.stream.JsonReader;
import com.google.gson.stream.JsonToken;
import com.google.gson.stream.JsonWriter;
import java.io.IOException;
import java.lang.reflect.ParameterizedType;
import java.lang.reflect.Type;
import java.util.Optional;

// Written using Claude Opus 4.6

/**
 * A Gson {@link TypeAdapterFactory} that handles serialization and deserialization of {@link
 * Optional} values.
 *
 * <ul>
 *   <li>{@code Optional.empty()} is written as JSON {@code null}.
 *   <li>{@code Optional.of(value)} is written as the serialized form of the inner value.
 *   <li>JSON {@code null} is read back as {@code Optional.empty()}.
 *   <li>Any other JSON value is deserialized and wrapped in {@code Optional.of(...)}.
 * </ul>
 *
 * <p>Register this factory on a {@code JSONSyncConfigBuilder} so that all {@code Optional<T>}
 * fields in constant classes are handled transparently.
 */
public class OptionalTypeAdapterFactory implements TypeAdapterFactory {

  @SuppressWarnings("unchecked")
  @Override
  public <T> TypeAdapter<T> create(Gson gson, TypeToken<T> type) {
    Class<? super T> rawType = type.getRawType();
    if (rawType != Optional.class) {
      return null; // This factory only handles Optional
    }

    // Resolve the inner type parameter (e.g. Optional<Pose2d> → Pose2d)
    Type innerType = getInnerType(type.getType());
    TypeAdapter<?> innerAdapter = gson.getAdapter(TypeToken.get(innerType));

    return (TypeAdapter<T>) new OptionalTypeAdapter<>(innerAdapter);
  }

  /**
   * Extracts the type argument from a parameterized {@code Optional<T>}.
   *
   * @param optionalType the parameterized type
   * @return the inner type {@code T}, or {@code Object.class} if raw
   */
  private static Type getInnerType(Type optionalType) {
    if (optionalType instanceof ParameterizedType parameterized) {
      return parameterized.getActualTypeArguments()[0];
    }
    // Raw Optional (no generic info) – fall back to Object
    return Object.class;
  }

  /**
   * TypeAdapter that converts between {@code Optional<E>} and JSON, delegating the inner value to
   * an existing adapter.
   */
  private static class OptionalTypeAdapter<E> extends TypeAdapter<Optional<E>> {
    private final TypeAdapter<E> innerAdapter;

    OptionalTypeAdapter(TypeAdapter<E> innerAdapter) {
      this.innerAdapter = innerAdapter;
    }

    @Override
    public void write(JsonWriter out, Optional<E> value) throws IOException {
      if (value == null || value.isEmpty()) {
        out.nullValue();
      } else {
        innerAdapter.write(out, value.get());
      }
    }

    @Override
    public Optional<E> read(JsonReader in) throws IOException {
      if (in.peek() == JsonToken.NULL) {
        in.nextNull();
        return Optional.empty();
      }
      return Optional.ofNullable(innerAdapter.read(in));
    }
  }
}
