package com.team581.trailblazer;

import com.team581.math.PoseErrorTolerance;
import java.util.List;
import java.util.Optional;

public class AutoSegmentBuilder {
  private final List<AutoPoint> points;
  private Optional<AutoConstraintOptions> constraints = Optional.empty();
  private Optional<PoseErrorTolerance> positionTolerance = Optional.empty();

  AutoSegmentBuilder(List<AutoPoint> waypoints) {
    this.points = waypoints;
  }

  private AutoSegment build() {
    return new AutoSegment(points, constraints, positionTolerance);
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
    this.positionTolerance = Optional.empty();
    return build();
  }

  /**
   * Builds the segment, which will be followed until the robot is within the given tolerance.
   *
   * @param tolerance The tolerance to use for the final point of the segment.
   * @return The segment.
   */
  public AutoSegment untilFinished(PoseErrorTolerance tolerance) {
    this.positionTolerance = Optional.of(tolerance);
    return build();
  }
}
