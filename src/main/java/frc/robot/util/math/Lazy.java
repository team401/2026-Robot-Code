package frc.robot.util.math;

import java.util.function.Supplier;

/**
 * The Lazy class lazily initializes an instance of another class by only creating it when it is
 * accessed using {@link #get()}.
 *
 * @param <T> the type of the instance to encapsulate
 */
public class Lazy<T> {
  private T instance = null;
  private final Supplier<T> initializer;

  /**
   * Create a new lazily evaluated object.
   *
   * @param initializer A supplier to create the instance when it is needed.
   */
  public Lazy(Supplier<T> initializer) {
    this.initializer = initializer;
  }

  /**
   * Get the instance stored in this Lazy, calling the initializer to create it if it has not yet
   * been instantiated.
   *
   * @return The instance stored in this Lazy
   */
  public T get() {
    if (instance == null) {
      instance = initializer.get();
    }

    return instance;
  }
}
