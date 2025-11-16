package com.team581.trailblazer.followers;

import com.team581.trailblazer.AutoPoint;
import com.team581.trailblazer.AutoSegment;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * A path follower calculates the robot speeds needed to drive from the current pose to some target
 * pose. This is distinct from a path tracker, which chooses what that target pose is.
 */
public interface PathFollower {
  /**
   * Calculate the robot speeds needed to drive the robot from the current pose to the target pose.
   *
   * @param currentPose The current pose of the robot.
   * @param targetPose The pose the robot should drive to.
   * @param currentPoint The current point being tracked (for constraint information).
   * @param allPoints All points in the path (for lookahead constraint information).
   * @param currentPointIndex The index of the current point being tracked.
   * @return The field relative chassis speeds the robot should drive at to reach the target pose.
   */
  public ChassisSpeeds calculateSpeeds(
      Pose2d currentPose,
      Pose2d targetPose,
      AutoPoint currentPoint,
      AutoSegment segment,
      int currentPointIndex);
}
