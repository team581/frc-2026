package com.team581.trailblazer;

import com.team581.math.PoseErrorTolerance;
import edu.wpi.first.math.geometry.Pose2d;
import java.util.List;
import java.util.Optional;

public record AutoSegment(
    List<AutoPoint> points,
    Optional<AutoConstraintOptions> constraints,
    Optional<PoseErrorTolerance> positionTolerance) {
  public Optional<AutoPoint> lastPoint() {
    if (points.isEmpty()) {
      return Optional.empty();
    }

    return Optional.of(points.get(points.size() - 1));
  }

  /**
   * Check if the robot is done following this segment.
   *
   * @param robotPose The current pose of the robot.
   * @param currentIndex The current index of the point being tracked.
   * @return Whether the robot is done following this segment.
   */
  public boolean atGoal(Pose2d robotPose, int currentIndex) {
    if (points.isEmpty()) {
      return true;
    }

    if (currentIndex != points.size() - 1) {
      // We aren't at the last point in the list, so we definitely aren't finished
      return false;
    }

    if (positionTolerance.isEmpty()) {
      return false;
    }

    var lastPoint = points.get(points.size() - 1);
    return positionTolerance
        .orElseThrow()
        .atPose(lastPoint.poseSupplier().get().getPose(), robotPose);
  }

  /**
   * Resolve the constraints for a point belonging to this segment.
   *
   * @param point The point to resolve the constraints for.
   * @return The constraints for the point.
   */
  public Optional<AutoConstraintOptions> getConstraints(AutoPoint point) {
    return constraints.or(this::constraints);
  }
}
