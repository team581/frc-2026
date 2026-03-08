package com.team581.simkit.internal;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;

public record ShotGamePiece(
    Translation3d start, Translation3d target, double tofSeconds, double createdAt) {
  private static final double GRAVITY = 9.81;

  public ShotGamePiece(Translation3d start, Translation3d target, double tofSeconds) {
    this(start, target, tofSeconds, Timer.getFPGATimestamp());
  }

  public boolean hasExistedFor(double durationSeconds) {
    return elapsedSeconds() > durationSeconds;
  }

  /** Compute the current position along the parametric trajectory. */
  public Translation3d pose() {
    var t = elapsedSeconds();
    var fraction = t / tofSeconds;

    // Horizontal: linear interpolation
    var x = MathUtil.interpolate(start.getX(), target.getX(), fraction);
    var y = MathUtil.interpolate(start.getY(), target.getY(), fraction);

    // Vertical: ballistic arc
    // vz0 chosen so that z(tofSeconds) = target.getZ()
    var vz0 = (target.getZ() - start.getZ() + 0.5 * GRAVITY * tofSeconds * tofSeconds) / tofSeconds;
    var z = start.getZ() + vz0 * t - 0.5 * GRAVITY * t * t;

    return new Translation3d(x, y, z);
  }

  private double elapsedSeconds() {
    return Timer.getFPGATimestamp() - createdAt;
  }
}
