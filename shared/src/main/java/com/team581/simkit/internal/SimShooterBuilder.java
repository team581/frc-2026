package com.team581.simkit.internal;

import com.google.errorprone.annotations.CanIgnoreReturnValue;
import java.util.ArrayList;
import java.util.List;
import java.util.OptionalDouble;
import java.util.function.Predicate;

public class SimShooterBuilder {
  private String logKey;
  private OptionalDouble maxGpLifetime = OptionalDouble.empty();
  private OptionalDouble minZ = OptionalDouble.empty();
  private final List<Predicate<ShotGamePiece>> filters = new ArrayList<>();

  public SimShooterBuilder(String name) {
    this.logKey = name;
  }

  public SimShooter build() {
    return new SimShooter(logKey, minZ, maxGpLifetime, filters);
  }

  @CanIgnoreReturnValue
  public SimShooterBuilder withLogKey(String logKey) {
    this.logKey = logKey;
    return this;
  }

  @CanIgnoreReturnValue
  public SimShooterBuilder withMaxGpLifetime(double maxGpLifetime) {
    this.maxGpLifetime = OptionalDouble.of(maxGpLifetime);
    return this;
  }

  @CanIgnoreReturnValue
  public SimShooterBuilder withMinZ(double minZ) {
    this.minZ = OptionalDouble.of(minZ);
    return this;
  }

  @CanIgnoreReturnValue
  public SimShooterBuilder withRemovalFunction(Predicate<ShotGamePiece> filter) {
    this.filters.add(filter);
    return this;
  }
}
