package com.team581.trailblazer;

import com.team581.autos.Point;
import com.team581.math.PoseErrorTolerance;
import edu.wpi.first.math.geometry.Pose2d;
import java.util.Optional;
import java.util.function.Supplier;

public record AutoPoint<T extends Enum<T>>(
    Supplier<Point> poseSupplier,
    Optional<AutoConstraintOptions> constraints,
    Optional<PoseErrorTolerance> transitionTolerance,
    Optional<T> marker) {
  public static AutoPoint<EmptyMarker> of(Point pose) {
    return new AutoPoint<EmptyMarker>(
        () -> pose, Optional.empty(), Optional.empty(), Optional.empty());
  }

  public static AutoPoint<EmptyMarker> of(Supplier<Point> poseSupplier) {
    return new AutoPoint<EmptyMarker>(
        poseSupplier, Optional.empty(), Optional.empty(), Optional.empty());
  }

  public static AutoPoint<EmptyMarker> ofBlue(Pose2d pose) {
    return of(Point.ofBlue(pose));
  }

  public static AutoPoint<EmptyMarker> ofRed(Pose2d pose) {
    return of(Point.ofRed(pose));
  }

  public AutoPoint<T> withLinearConstraints(double maxVelocity, double maxAcceleration) {
    return new AutoPoint<T>(
        poseSupplier,
        Optional.of(
            constraints
                .orElseGet(AutoConstraintOptions::new)
                .withLinearConstraints(maxVelocity, maxAcceleration)),
        transitionTolerance,
        marker);
  }

  public AutoPoint<T> withAngularConstraints(
      double maxAngularVelocity, double maxAngularAcceleration) {
    return new AutoPoint<T>(
        poseSupplier,
        Optional.of(
            constraints
                .orElseGet(AutoConstraintOptions::new)
                .withAngularConstraints(maxAngularVelocity, maxAngularAcceleration)),
        transitionTolerance,
        marker);
  }

  public AutoPoint<T> withTransitionTolerance(PoseErrorTolerance transitionTolerance) {
    return new AutoPoint<T>(poseSupplier, constraints, Optional.of(transitionTolerance), marker);
  }

  public <E extends Enum<E>> AutoPoint<E> withMarker(E marker) {
    return new AutoPoint<E>(poseSupplier, constraints, transitionTolerance, Optional.of(marker));
  }
}
