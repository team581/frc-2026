package com.team581.refaster;

import com.google.errorprone.refaster.annotation.AfterTemplate;
import com.google.errorprone.refaster.annotation.BeforeTemplate;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.OptionalInt;
import java.util.OptionalLong;

/**
 * Refaster rules to prefer primitive optional types ({@link OptionalDouble}, {@link OptionalInt},
 * {@link OptionalLong}) over their boxed counterparts ({@code Optional<Double>}, etc.).
 *
 * <p>Using primitive optionals avoids unnecessary boxing/unboxing overhead and makes the intent
 * clearer.
 *
 * <p>Note: Java does not have {@code OptionalFloat} or {@code OptionalShort} in the standard
 * library.
 */
class OptionalRules {
  /** Prefer {@link OptionalDouble#empty()} over {@code Optional.<Double>empty()}. */
  static class OptionalDoubleEmpty {
    @BeforeTemplate
    Optional<Double> before() {
      return Optional.<Double>empty();
    }

    @AfterTemplate
    OptionalDouble replacement() {
      return OptionalDouble.empty();
    }
  }

  /** Prefer {@link OptionalDouble#of(double)} over {@code Optional.of(Double)}. */
  static class OptionalDoubleOf {
    @BeforeTemplate
    Optional<Double> before(double value) {
      return Optional.of(value);
    }

    @AfterTemplate
    OptionalDouble replacement(double value) {
      return OptionalDouble.of(value);
    }
  }

  /** Prefer {@link OptionalInt#empty()} over {@code Optional.<Integer>empty()}. */
  static class OptionalIntEmpty {
    @BeforeTemplate
    Optional<Integer> before() {
      return Optional.<Integer>empty();
    }

    @AfterTemplate
    OptionalInt replacement() {
      return OptionalInt.empty();
    }
  }

  /** Prefer {@link OptionalInt#of(int)} over {@code Optional.of(Integer)}. */
  static class OptionalIntOf {
    @BeforeTemplate
    Optional<Integer> before(int value) {
      return Optional.of(value);
    }

    @AfterTemplate
    OptionalInt replacement(int value) {
      return OptionalInt.of(value);
    }
  }

  /** Prefer {@link OptionalLong#empty()} over {@code Optional.<Long>empty()}. */
  static class OptionalLongEmpty {
    @BeforeTemplate
    Optional<Long> before() {
      return Optional.<Long>empty();
    }

    @AfterTemplate
    OptionalLong replacement() {
      return OptionalLong.empty();
    }
  }

  /** Prefer {@link OptionalLong#of(long)} over {@code Optional.of(Long)}. */
  static class OptionalLongOf {
    @BeforeTemplate
    Optional<Long> before(long value) {
      return Optional.of(value);
    }

    @AfterTemplate
    OptionalLong replacement(long value) {
      return OptionalLong.of(value);
    }
  }

  private OptionalRules() {}
}
