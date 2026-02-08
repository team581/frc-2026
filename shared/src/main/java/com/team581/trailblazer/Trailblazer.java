package com.team581.trailblazer;

import com.team581.trailblazer.followers.PathFollower;
import com.team581.trailblazer.segments.AutoSegment;
import com.team581.trailblazer.trackers.PathTracker;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.List;
import java.util.Optional;
import org.jspecify.annotations.Nullable;

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
  @SafeVarargs
  public static AutoSegmentBuilder segment(AutoPoint<?>... waypoints) {
    return new AutoSegmentBuilder(List.of(waypoints));
  }

  private final PathTracker pathTracker;
  private final PathFollower pathFollower;
  private int currentIndex = -1;
  private Optional<AutoSegment> currentSegment = Optional.empty();
  private boolean needsFollowerReset = false;

  public Trailblazer(PathTracker pathTracker, PathFollower pathFollower) {
    this.pathTracker = pathTracker;
    this.pathFollower = pathFollower;
  }

  public boolean atGoal(Pose2d currentPose) {
    return currentSegment.filter(segment -> segment.atGoal(currentPose, currentIndex)).isPresent();
  }

  public boolean atGoal(Translation2d currentTranslation) {
    return currentSegment
        .filter(segment -> segment.atGoal(currentTranslation, currentIndex))
        .isPresent();
  }

  public ChassisSpeeds getFieldRelativeSetpoint(
      Pose2d currentPose, ChassisSpeeds currentFieldRelativeSpeeds) {
    return getFieldRelativeSetpoint(currentPose, currentFieldRelativeSpeeds, null);
  }

  public ChassisSpeeds getFieldRelativeSetpoint(
      Pose2d currentPose,
      ChassisSpeeds currentFieldRelativeSpeeds,
      @Nullable Rotation2d trackerRotationOverride) {
    if (currentSegment.isEmpty()) {
      return new ChassisSpeeds();
    }

    // Reset the path follower's constraint calculator when starting a new segment.
    // This ensures the profiled controller knows the actual current angular velocity.
    // Technically you could call reset over and over, but that creates a lot of objects and may
    // cause jerky motion since we are effectively replanning a trajectory over and over.
    if (needsFollowerReset) {
      pathFollower.reset(currentFieldRelativeSpeeds, currentPose.getRotation().getRadians());
      needsFollowerReset = false;
    }

    var segment = currentSegment.orElseThrow();

    // Update tracker with current robot state
    pathTracker.updateRobotState(currentPose, currentFieldRelativeSpeeds);

    // Update current index from tracker
    currentIndex = pathTracker.getCurrentPointIndex();
    DogLog.log("Trailblazer/Tracker/CurrentIndex", currentIndex);

    var targetPose = pathTracker.getTargetPose(trackerRotationOverride);
    DogLog.log("Trailblazer/Tracker/TargetPose", targetPose);

    // Calculate speeds using follower
    return pathFollower.calculateSpeeds(
        currentFieldRelativeSpeeds,
        currentPose,
        targetPose,
        segment.points.get(currentIndex),
        segment,
        currentIndex);
  }

  /**
   * Check if the robot has passed or reached the point with the given marker in the current
   * segment.
   *
   * @param marker The marker to check.
   * @return Whether the robot has passed the marker. Returns false if no segment is active or if
   *     the marker enum type doesn't match the current segment's marker type.
   */
  public boolean passedMarker(Enum<?> marker) {
    if (currentSegment.isEmpty()) {
      return false;
    }

    return currentSegment.orElseThrow().passedMarker(currentIndex, marker);
  }

  public void setActiveSegment(AutoSegment segment) {
    if (currentSegment.isPresent() && currentSegment.orElseThrow().equals(segment)) {
      return;
    }

    currentSegment = Optional.of(segment);
    pathTracker.resetAndSetPoints(segment.points);
    currentIndex = pathTracker.getCurrentPointIndex();
    needsFollowerReset = true;

    DogLog.log(
        "Trailblazer/Tracker/InitialSegmentPoints",
        segment.points.stream()
            .map(point -> point.poseSupplier().get().getPose())
            .toArray(Pose2d[]::new));
  }
}
