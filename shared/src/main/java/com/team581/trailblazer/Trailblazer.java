package com.team581.trailblazer;

import com.team581.autos.Point;
import com.team581.trailblazer.followers.PathFollower;
import com.team581.trailblazer.trackers.PathTracker;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.Optional;

/**
 * Trailblazer is Team 581's custom path following library. We built Trailblazer to give us end to
 * end control over every aspect of how our autos execute.
 *
 * <p>Trailblazer is made up of a few components:
 *
 * <ol>
 *   <li>Path segments, which are a list of points to follow with the robot
 *   <li>Can include constraints on robot motion per point or per segment
 *   <li>Path trackers, which determine the pose setpoint for the robot
 *   <li>Path followers, which calculate a velocity setpoint to reach the pose setpoint
 * </ol>
 */
public class Trailblazer {
  public static AutoSegmentBuilder segment(Point... waypoints) {
    return new AutoSegmentBuilder(waypoints);
  }

  private final LocalizationBase localization;
  private final PathTracker pathTracker;
  private final PathFollower pathFollower;
  private int currentIndex = -1;
  private Optional<AutoSegment> currentSegment = Optional.empty();

  public Trailblazer(
      LocalizationBase localization, PathTracker pathTracker, PathFollower pathFollower) {
    this.localization = localization;
    this.pathTracker = pathTracker;
    this.pathFollower = pathFollower;
  }

  public void followSegment(AutoSegment segment) {
    if (currentSegment.isPresent() && currentSegment.orElseThrow().equals(segment)) {
      return;
    }

    currentSegment = Optional.of(segment);
    currentIndex = 0;
    pathTracker.resetAndSetPoints(segment.points());
  }

  public boolean atGoal() {
    return currentSegment
        .map(segment -> segment.atGoal(localization.getPose(), currentIndex))
        .orElse(false);
  }

  public ChassisSpeeds getFieldRelativeSetpoint(ChassisSpeeds currentFieldRelativeSpeeds) {
    if (currentSegment.isEmpty()) {
      return new ChassisSpeeds();
    }

    var segment = currentSegment.orElseThrow();
    var currentPose = localization.getPose();

    // Update tracker with current robot state
    pathTracker.updateRobotState(currentPose, currentFieldRelativeSpeeds);

    // Update current index from tracker
    currentIndex = pathTracker.getCurrentPointIndex();

    // Calculate speeds using follower
    return pathFollower.calculateSpeeds(
        currentPose,
        pathTracker.getTargetPose(),
        segment.points().get(currentIndex),
        segment,
        currentIndex);
  }
}
