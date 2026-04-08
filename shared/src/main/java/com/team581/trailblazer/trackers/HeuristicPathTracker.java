package com.team581.trailblazer.trackers;

import com.google.common.collect.ImmutableList;
import com.team581.math.PoseErrorTolerance;
import com.team581.trailblazer.AutoPoint;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.List;
import org.jspecify.annotations.Nullable;

public class HeuristicPathTracker implements PathTracker {
  private final PoseErrorTolerance defaultTransitionTolerance;
  private List<AutoPoint<?>> points = ImmutableList.of();
  private Pose2d currentPose = Pose2d.kZero;
  private int currentPointIndex = 0;
  private double maxT = 0;

  /**
   * The pose where the robot was when it transitioned to the current point. Used as the Bezier
   * start point for arc segments, so that t=0 at transition and there is no rotation jump.
   */
  private Pose2d transitionPose = Pose2d.kZero;

  public HeuristicPathTracker(PoseErrorTolerance defaultTransitionTolerance) {
    this.defaultTransitionTolerance = defaultTransitionTolerance;
  }

  @Override
  public int getCurrentPointIndex() {
    return currentPointIndex;
  }

  @Override
  public Pose2d getTargetPose(@Nullable Rotation2d trackerRotationOverride) {
    if (currentPointIndex < points.size() - 1) {
      var currentPoint = points.get(currentPointIndex);
      var toleranceCheckPose = currentPoint.getPose();

      if (trackerRotationOverride != null) {
        toleranceCheckPose =
            new Pose2d(toleranceCheckPose.getTranslation(), trackerRotationOverride);
      }

      var usedTolerance = currentPoint.transitionTolerance().orElse(defaultTransitionTolerance);

      DogLog.log("Trailblazer/Tracker/TransitionTolerance", usedTolerance);

      if (usedTolerance.atPose(toleranceCheckPose, currentPose)) {
        currentPointIndex++;
        maxT = 0;
        transitionPose = currentPose;
      }
    }

    var currentPoint = points.get(currentPointIndex);
    var targetPose = currentPoint.getPose();

    if (currentPointIndex > 0) {
      var isArc = currentPoint.arcMidpoint().isPresent();
      var previousPose = points.get(currentPointIndex - 1).getPose();
      // For translation: arcs use transition pose (where robot actually was) so t starts at 0,
      // straight lines use previous waypoint
      var translationStart =
          isArc ? transitionPose.getTranslation() : previousPose.getTranslation();

      double distanceFromStart = currentPose.getTranslation().getDistance(translationStart);
      double distanceToTarget =
          currentPose.getTranslation().getDistance(targetPose.getTranslation());
      double totalTravelDistance = distanceFromStart + distanceToTarget;

      if (totalTravelDistance > 0) {
        // t=0 at start, t=1 at target point
        // Ratchet forward only to prevent jitter from noise/vision updates
        double t = distanceFromStart / totalTravelDistance;
        t = Math.max(0, Math.min(1, t));
        maxT = Math.max(maxT, t);
        t = maxT;

        var targetTranslation = targetPose.getTranslation();

        if (isArc) {
          var arcMid = currentPoint.arcMidpoint().orElseThrow();
          var p0 = translationStart;
          var p1 = arcMid.getTranslation();
          var p2 = targetPose.getTranslation();
          // Quadratic Bezier: B(t) = (1-t)^2 * P0 + 2(1-t)t * P1 + t^2 * P2
          var oneMinusT = 1 - t;
          targetTranslation =
              p0.times(oneMinusT * oneMinusT)
                  .plus(p1.times(2 * oneMinusT * t))
                  .plus(p2.times(t * t));
        }

        // Use a fixed rotation target — the follower is responsible for pacing rotation via
        // a dynamic angular velocity cap, rather than the tracker interpolating an intermediate
        // heading. For arcs, we output the midpoint rotation as a stable target for the first
        // half, then switch to the final rotation for the second half. This disambiguates the
        // rotation direction without creating a continuously moving setpoint that the PID
        // would lag behind.
        Rotation2d targetRotation;
        if (isArc) {
          var midRotation = currentPoint.arcMidpoint().orElseThrow().getRotation();
          targetRotation = t < 0.5 ? midRotation : targetPose.getRotation();
        } else {
          targetRotation = targetPose.getRotation();
        }

        targetPose = new Pose2d(targetTranslation, targetRotation);
      }
    }

    return targetPose;
  }

  @Override
  public void resetAndSetPoints(List<AutoPoint<?>> points) {
    this.resetAndSetPoints(points, 0);
  }

  @Override
  public void resetAndSetPoints(List<AutoPoint<?>> points, int index) {
    this.points = points;
    this.currentPointIndex = index;
    this.maxT = 0;
    this.transitionPose = currentPose;
  }

  @Override
  public void updateRobotState(Pose2d currentPose, ChassisSpeeds currentFieldRelativeRobotSpeeds) {
    this.currentPose = currentPose;
  }
}
