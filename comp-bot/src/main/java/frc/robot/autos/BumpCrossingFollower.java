package frc.robot.autos;

import com.team581.mechanisms.imu.BumpCrossingState;
import com.team581.mechanisms.imu.BumpCrossingTracker;
import com.team581.trailblazer.AutoPoint;
import com.team581.trailblazer.followers.PathFollower;
import com.team581.trailblazer.segments.AutoSegment;
import com.team581.util.FmsUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Wraps a {@link PathFollower} but injects custom velocities to ensure consistent bump crossing
 * behavior.
 */
public class BumpCrossingFollower implements PathFollower {
  private static final double CROSSING_UPHILL_LINEAR_VELOCITY = 3;
  private static final double CROSSING_DOWNHILL_LINEAR_VELOCITY = 1;

  private final PathFollower follower;

  private final BumpCrossingTracker bumpCrossingTracker;

  public BumpCrossingFollower(PathFollower follower, BumpCrossingTracker bumpCrossingTracker) {
    this.follower = follower;
    this.bumpCrossingTracker = bumpCrossingTracker;
  }

  @Override
  public ChassisSpeeds calculateSpeeds(
      ChassisSpeeds currentSpeeds,
      Pose2d currentPose,
      Pose2d targetPose,
      AutoPoint<?> currentPoint,
      AutoSegment segment,
      int currentPointIndex) {
    bumpCrossingTracker.setCurrentSpeeds(currentSpeeds);

    var originalSpeeds =
        follower.calculateSpeeds(
            currentSpeeds, currentPose, targetPose, currentPoint, segment, currentPointIndex);

    var state = bumpCrossingTracker.getState();

    if (state == BumpCrossingState.NOT_ON_BUMP) {
      return originalSpeeds;
    }

    var unsignedXVelocity =
        state == BumpCrossingState.CROSSING_UPHILL
            ? CROSSING_UPHILL_LINEAR_VELOCITY
            : CROSSING_DOWNHILL_LINEAR_VELOCITY;

    var xVelocity = FmsUtil.isRedAlliance() ? unsignedXVelocity : -unsignedXVelocity;

    // We preserve the Y and angular speeds to ensure we stay centered with the bump and maintain
    // the right heading
    return new ChassisSpeeds(
        xVelocity, originalSpeeds.vyMetersPerSecond, originalSpeeds.omegaRadiansPerSecond);
  }

  @Override
  public void reset(Pose2d currentPose, ChassisSpeeds currentSpeeds) {
    follower.reset(currentPose, currentSpeeds);
  }
}
