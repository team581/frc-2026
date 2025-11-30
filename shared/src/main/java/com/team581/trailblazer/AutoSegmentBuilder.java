package com.team581.trailblazer;

import java.util.List;
import java.util.NoSuchElementException;
import java.util.Optional;

public class AutoSegmentBuilder {
  private final List<AutoPoint> points;
  private Optional<AutoConstraintOptions> constraints = Optional.empty();

  AutoSegmentBuilder(List<AutoPoint> waypoints) {
    this.points = waypoints;
  }

  private AutoSegment build(AutoSegmentEndBehavior endBehavior) {
    return new AutoSegment(points, constraints, endBehavior);
  }

  /**
   * Set the linear constraints for the segment.
   *
   * @param maxVelocity The maximum velocity in meters per second.
   * @param maxAcceleration The maximum acceleration in meters per second squared.
   * @return This builder.
   */
  public AutoSegmentBuilder withLinearConstraints(double maxVelocity, double maxAcceleration) {
    this.constraints =
        Optional.of(
            constraints
                .orElseGet(AutoConstraintOptions::new)
                .withLinearConstraints(maxVelocity, maxAcceleration));
    return this;
  }

  /**
   * Set the angular constraints for the segment.
   *
   * @param maxAngularVelocity The maximum angular velocity in radians per second.
   * @param maxAngularAcceleration The maximum angular acceleration in radians per second squared.
   * @return This builder.
   */
  public AutoSegmentBuilder withAngularConstraints(
      double maxAngularVelocity, double maxAngularAcceleration) {
    this.constraints =
        Optional.of(
            constraints
                .orElseGet(AutoConstraintOptions::new)
                .withAngularConstraints(maxAngularVelocity, maxAngularAcceleration));
    return this;
  }

  /**
   * Builds the segment, which will be followed indefinitely.
   *
   * @return The segment.
   */
  public AutoSegment forever() {
    return build(AutoSegmentEndBehavior.FOREVER);
  }

  /**
   * Builds the segment, which will be followed until the robot is within the tolerance of the last
   * point.
   *
   * @return The segment.
   */
  public AutoSegment untilFinished() {
    if (points.isEmpty()) {
      return forever();
    }

    var lastPoint = points.get(points.size() - 1);

    if (lastPoint.transitionTolerance().isEmpty()) {
      throw new NoSuchElementException(
          "Last point is missing a transition tolerance, but segment is trying to be built with untilFinished()");
    }

    return build(AutoSegmentEndBehavior.LAST_POINT_TRANSITION_TOLERANCE);
  }
}
