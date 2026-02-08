package com.team581.trailblazer.trackers;

import com.team581.trailblazer.AutoPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.List;
import org.jspecify.annotations.Nullable;

/**
 * A path tracker takes a list of points making up a path, and decides which intermediary point
 * along the path to drive to next.
 */
public interface PathTracker {
  /**
   * Get the index of the input auto point that is currently most relevant to whatever the tracker
   * is doing. Used for triggering side effects.
   *
   * @return The index point being tracked.
   */
  public int getCurrentPointIndex();

  /**
   * Calculate the pose the robot should drive to next, based on its progress following the path.
   *
   * @return The pose the robot should drive to next.
   */
  public Pose2d getTargetPose(@Nullable Rotation2d trackerRotationOverride);

  /**
   * Reset the state of the path tracker, as well as set the new list of points to consider in
   * {@link #getTargetPose()}.
   *
   * @param points The new list of points to store.
   */
  public void resetAndSetPoints(List<? extends AutoPoint<?>> points);

  /**
   * Runs once per loop cycle to update the tracker with the current robot state.
   *
   * @param currentPose The current pose of the robot.
   * @param currentFieldRelativeRobotSpeeds The current field relative speeds of the robot.
   */
  public void updateRobotState(Pose2d currentPose, ChassisSpeeds currentFieldRelativeRobotSpeeds);
}
