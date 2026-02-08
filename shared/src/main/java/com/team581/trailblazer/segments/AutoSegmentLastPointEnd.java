package com.team581.trailblazer.segments;

import com.team581.trailblazer.AutoConstraintOptions;
import com.team581.trailblazer.AutoPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import java.util.Optional;

public class AutoSegmentLastPointEnd extends AutoSegment {
  public AutoSegmentLastPointEnd(
      List<AutoPoint<?>> points, Optional<AutoConstraintOptions> constraints) {
    super(points, constraints);
  }

  @Override
  public boolean atGoal(Pose2d robotPose, int currentIndex) {
    if (points.isEmpty()) {
      return true;
    }

    if (currentIndex != points.size() - 1) {
      // We aren't at the last point in the list, so we definitely aren't finished
      return false;
    }

    return points
        .get(points.size() - 1)
        .transitionTolerance()
        .orElseThrow()
        .atPose(points.get(points.size() - 1).poseSupplier().get().getPose(), robotPose);
  }

  @Override
  public boolean atGoal(Translation2d robotTranslation, int currentIndex) {
    if (points.isEmpty()) {
      return true;
    }

    if (currentIndex != points.size() - 1) {
      // We aren't at the last point in the list, so we definitely aren't finished
      return false;
    }

    return points
        .get(points.size() - 1)
        .transitionTolerance()
        .orElseThrow()
        .atTranslation(
            points.get(points.size() - 1).poseSupplier().get().getPose().getTranslation(),
            robotTranslation);
  }
}
