package com.team581.trailblazer.trackers;

import com.google.common.collect.ImmutableList;
import com.team581.math.PoseErrorTolerance;
import com.team581.trailblazer.AutoPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.List;
import org.jspecify.annotations.Nullable;

public class HeuristicPathTracker implements PathTracker {
  private final PoseErrorTolerance defaultTransitionTolerance;
  private List<? extends AutoPoint<?>> points = ImmutableList.of();
  private Pose2d currentPose = Pose2d.kZero;
  private int currentPointIndex = 0;

  public HeuristicPathTracker(PoseErrorTolerance defaultTransitionTolerance) {
    this.defaultTransitionTolerance = defaultTransitionTolerance;
  }

  @Override
  public int getCurrentPointIndex() {
    return currentPointIndex;
  }

  @Override
  public Pose2d getTargetPose(@Nullable Rotation2d trackerRotationOverride) {
    var currentPoint = points.get(getCurrentPointIndex());
    var currentTargetPose = currentPoint.poseSupplier().get().getPose();

    if (trackerRotationOverride != null) {
      currentTargetPose = new Pose2d(currentTargetPose.getTranslation(), trackerRotationOverride);
    }

    if (currentPointIndex < points.size() - 1
        && currentPoint
            .transitionTolerance()
            .orElse(defaultTransitionTolerance)
            .atPose(currentTargetPose, currentPose)) {
      currentPointIndex++;
    }

    return points.get(currentPointIndex).poseSupplier().get().getPose();
  }

  @Override
  public void resetAndSetPoints(List<? extends AutoPoint<?>> points) {
    this.points = points;
    this.currentPointIndex = 0;
  }

  @Override
  public void updateRobotState(Pose2d currentPose, ChassisSpeeds currentFieldRelativeRobotSpeeds) {
    this.currentPose = currentPose;
  }
}
