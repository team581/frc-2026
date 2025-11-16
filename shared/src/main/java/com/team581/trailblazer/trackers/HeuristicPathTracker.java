package com.team581.trailblazer.trackers;

import com.google.common.collect.ImmutableList;
import com.team581.math.PoseErrorTolerance;
import com.team581.trailblazer.AutoPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.List;

public class HeuristicPathTracker implements PathTracker {
  private final PoseErrorTolerance defaultTransitionTolerance;
  private List<AutoPoint> points = ImmutableList.of();
  private Pose2d currentPose = Pose2d.kZero;
  private int currentPointIndex = 0;

  public HeuristicPathTracker(PoseErrorTolerance defaultTransitionTolerance) {
    this.defaultTransitionTolerance = defaultTransitionTolerance;
  }

  @Override
  public void resetAndSetPoints(List<AutoPoint> points) {
    this.points = points;
  }

  @Override
  public void updateRobotState(Pose2d currentPose, ChassisSpeeds currentFieldRelativeRobotSpeeds) {
    this.currentPose = currentPose;
  }

  @Override
  public Pose2d getTargetPose() {
    var currentPoint = points.get(getCurrentPointIndex());
    var currentTargetPose = currentPoint.poseSupplier().get().getPose();

    if (currentPointIndex < points.size() - 1
        && currentPoint
            .positionTolerance()
            .orElse(defaultTransitionTolerance)
            .atPose(currentTargetPose, currentPose)) {
      currentPointIndex++;
    }

    return points.get(currentPointIndex).poseSupplier().get().getPose();
  }

  @Override
  public int getCurrentPointIndex() {
    return currentPointIndex;
  }
}
