package frc.robot.util;

import java.util.Optional;
import java.util.function.BiConsumer;
import java.util.function.BiFunction;

/**
 * The OptionalUtil class provides static methods to make it easier/more convenient to work with
 * multiple optionals at a time.
 */
public class OptionalUtil {
  private OptionalUtil() {}

  public static <A, B> void ifPresent(
      Optional<A> firstOptional, Optional<B> secondOptional, BiConsumer<A, B> function) {
    firstOptional.ifPresent(
        first -> secondOptional.ifPresent(second -> function.accept(first, second)));
  }

  public static <A, B, C> Optional<C> map(
      Optional<A> firstOptional, Optional<B> secondOptional, BiFunction<A, B, C> mapper) {
    return firstOptional.flatMap(
        first -> secondOptional.map(second -> mapper.apply(first, second)));
  }
}
