package com.team581.trailblazer.segments;

import com.team581.math.PoseErrorTolerance;
import com.team581.trailblazer.AutoConstraintOptions;
import com.team581.trailblazer.AutoPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import java.util.Optional;

public class AutoSegmentCustomEnd extends AutoSegment {
  private final PoseErrorTolerance finishedTolerance;

  public AutoSegmentCustomEnd(
      List<AutoPoint<?>> points,
      Optional<AutoConstraintOptions> constraints,
      PoseErrorTolerance finishedTolerance) {
    super(points, constraints);
    this.finishedTolerance = finishedTolerance;
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

    return finishedTolerance.atPose(
        points.get(points.size() - 1).poseSupplier().get().getPose(), robotPose);
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

    return finishedTolerance.atTranslation(
        points.get(points.size() - 1).poseSupplier().get().getPose().getTranslation(),
        robotTranslation);
  }
}
