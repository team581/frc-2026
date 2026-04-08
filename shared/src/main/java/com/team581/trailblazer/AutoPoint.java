package com.team581.trailblazer;

import com.team581.autos.Point;
import com.team581.math.PoseErrorTolerance;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.Optional;
import java.util.function.Supplier;

public record AutoPoint<T extends Enum<T>>(
    Supplier<Point> poseSupplier,
    Optional<LinearConstraintOptions> linearConstraints,
    Optional<AngularConstraintOptions> angularConstraints,
    Optional<PoseErrorTolerance> transitionTolerance,
    Optional<T> marker,
    Optional<Point> arcMidpoint) {
  public static AutoPoint<EmptyMarker> of(Point pose) {
    return new AutoPoint<EmptyMarker>(
        () -> pose,
        Optional.empty(),
        Optional.empty(),
        Optional.empty(),
        Optional.empty(),
        Optional.empty());
  }

  public static AutoPoint<EmptyMarker> of(Supplier<Point> poseSupplier) {
    return new AutoPoint<EmptyMarker>(
        poseSupplier,
        Optional.empty(),
        Optional.empty(),
        Optional.empty(),
        Optional.empty(),
        Optional.empty());
  }

  public static AutoPoint<EmptyMarker> ofBlue(Pose2d pose) {
    return of(Point.ofBlue(pose));
  }

  public static AutoPoint<EmptyMarker> ofRed(Pose2d pose) {
    return of(Point.ofRed(pose));
  }

  public Pose2d getPose() {
    return poseSupplier.get().getPose();
  }

  public AutoPoint<T> withLinearConstraints(double maxVelocity, double maxAcceleration) {
    return new AutoPoint<T>(
        poseSupplier,
        Optional.of(new LinearConstraintOptions(maxVelocity, maxAcceleration)),
        angularConstraints,
        transitionTolerance,
        marker,
        arcMidpoint);
  }

  public AutoPoint<T> withAngularConstraints(
      double maxAngularVelocity, double maxAngularAcceleration) {
    return new AutoPoint<T>(
        poseSupplier,
        linearConstraints,
        Optional.of(new AngularConstraintOptions(maxAngularVelocity, maxAngularAcceleration)),
        transitionTolerance,
        marker,
        arcMidpoint);
  }

  public AutoPoint<T> withTransitionTolerance(PoseErrorTolerance transitionTolerance) {
    return new AutoPoint<T>(
        poseSupplier,
        linearConstraints,
        angularConstraints,
        Optional.of(transitionTolerance),
        marker,
        arcMidpoint);
  }

  public <E extends Enum<E>> AutoPoint<E> withMarker(E marker) {
    return new AutoPoint<E>(
        poseSupplier,
        linearConstraints,
        angularConstraints,
        transitionTolerance,
        Optional.of(marker),
        arcMidpoint);
  }

  public AutoPoint<T> withArcMidpoint(Point arcMidpoint) {
    return new AutoPoint<T>(
        poseSupplier,
        linearConstraints,
        angularConstraints,
        transitionTolerance,
        marker,
        Optional.of(arcMidpoint));
  }

  /**
   * Configures this point to be reached via a curved arc instead of a straight line. The arc's
   * Bezier control point is computed automatically from the previous point, this point, and the
   * given extension offsets. The midpoint rotation is set to {@link Rotation2d#kZero}.
   *
   * <p>The control point is placed at the midpoint between {@code arcStart} and this point's
   * position, offset by the given X and Y extension distances.
   *
   * <p>Example: if the previous point is at (9.5, 4.4) and this point is at (10.575, 4.4), calling
   * {@code withArcExtension(prevPoint, 0, -0.51)} places the control point at (10.0375, 3.89).
   *
   * @param arcStart The point the robot is coming from (the previous waypoint in the segment).
   * @param xExtensionMeters How far to offset the control point from the midpoint in X (meters).
   * @param yExtensionMeters How far to offset the control point from the midpoint in Y (meters).
   * @return A new AutoPoint with the computed arc midpoint.
   */
  public AutoPoint<T> withArcExtension(
      Point arcStart, double xExtensionMeters, double yExtensionMeters) {
    return withArcExtension(arcStart, xExtensionMeters, yExtensionMeters, Rotation2d.kZero);
  }

  /**
   * Configures this point to be reached via a curved arc instead of a straight line. The arc's
   * Bezier control point is computed automatically from the previous point, this point, and the
   * given extension offsets, with a custom midpoint rotation.
   *
   * <p>The control point is placed at the midpoint between {@code arcStart} and this point's
   * position, offset by the given X and Y extension distances.
   *
   * @param arcStart The point the robot is coming from (the previous waypoint in the segment).
   * @param xExtensionMeters How far to offset the control point from the midpoint in X (meters).
   * @param yExtensionMeters How far to offset the control point from the midpoint in Y (meters).
   * @param midpointRotation The rotation to use at the arc midpoint.
   * @return A new AutoPoint with the computed arc midpoint.
   */
  public AutoPoint<T> withArcExtension(
      Point arcStart,
      double xExtensionMeters,
      double yExtensionMeters,
      Rotation2d midpointRotation) {
    // Resolve this point's pose supplier eagerly to compute the midpoint at construction time.
    // This is safe because segment definitions use static poses.
    var endPose = poseSupplier.get();

    double midX = (arcStart.redPose().getX() + endPose.redPose().getX()) / 2 + xExtensionMeters;
    double midY = (arcStart.redPose().getY() + endPose.redPose().getY()) / 2 + yExtensionMeters;

    var computedMidpoint = Point.ofRed(new Pose2d(midX, midY, midpointRotation));

    return new AutoPoint<T>(
        poseSupplier,
        linearConstraints,
        angularConstraints,
        transitionTolerance,
        marker,
        Optional.of(computedMidpoint));
  }
}
