package com.team581.trailblazer;

import com.team581.autos.Point;
import com.team581.math.PoseErrorTolerance;
import java.util.Optional;
import java.util.function.Supplier;

public record AutoPoint(
    Supplier<Point> poseSupplier,
    Optional<AutoConstraintOptions> constraints,
    Optional<PoseErrorTolerance> positionTolerance) {
  public static AutoPoint of(Point pose) {
    return new AutoPoint(() -> pose, Optional.empty(), Optional.empty());
  }

  public static AutoPoint of(Supplier<Point> poseSupplier) {
    return new AutoPoint(poseSupplier, Optional.empty(), Optional.empty());
  }

  public AutoPoint withLinearConstraints(double maxVelocity, double maxAcceleration) {
    return new AutoPoint(
        poseSupplier,
        Optional.of(
            constraints
                .orElseGet(AutoConstraintOptions::new)
                .withLinearConstraints(maxVelocity, maxAcceleration)),
        positionTolerance);
  }

  public AutoPoint withAngularConstraints(
      double maxAngularVelocity, double maxAngularAcceleration) {
    return new AutoPoint(
        poseSupplier,
        Optional.of(
            constraints
                .orElseGet(AutoConstraintOptions::new)
                .withAngularConstraints(maxAngularVelocity, maxAngularAcceleration)),
        positionTolerance);
  }

  public AutoPoint withPositionTolerance(PoseErrorTolerance positionTolerance) {
    return new AutoPoint(poseSupplier, constraints, Optional.of(positionTolerance));
  }
}
