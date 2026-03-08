package com.team581.simkit.internal;

import static java.util.stream.Collectors.toCollection;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.ArrayDeque;
import java.util.List;
import java.util.OptionalDouble;
import java.util.Queue;
import java.util.function.Predicate;

public class SimShooter {
  private final Predicate<ShotGamePiece> shouldKeep;
  private final String logKey;

  private Queue<ShotGamePiece> shots = new ArrayDeque<>();

  SimShooter(
      String logKey,
      OptionalDouble minZ,
      OptionalDouble maxGpLifetime,
      List<Predicate<ShotGamePiece>> additionalFilters) {
    this.logKey = logKey;

    // Always remove shots that have reached their target (exceeded their time of flight)
    Predicate<ShotGamePiece> shouldRemove = gp -> gp.hasExistedFor(gp.tofSeconds());

    if (minZ.isPresent()) {
      shouldRemove = shouldRemove.or(gp -> gp.pose().getZ() < minZ.getAsDouble());
    }

    if (maxGpLifetime.isPresent()) {
      shouldRemove = shouldRemove.or(gp -> gp.hasExistedFor(maxGpLifetime.getAsDouble()));
    }

    for (Predicate<ShotGamePiece> additionalFilter : additionalFilters) {
      shouldRemove = shouldRemove.or(additionalFilter);
    }

    shouldKeep = shouldRemove.negate();
  }

  public void shoot(Translation3d start, Translation3d target, double tofSeconds) {
    shots.add(new ShotGamePiece(start, target, tofSeconds));
  }

  public void update() {
    shots = shots.stream().filter(shouldKeep).collect(toCollection(ArrayDeque::new));

    DogLog.log(logKey, shots.stream().map(ShotGamePiece::pose).toArray(Translation3d[]::new));
  }
}
