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

  // TODO: Actually use this
  @SuppressWarnings("unused")
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
    var originalSpeeds =
        follower.calculateSpeeds(
            currentSpeeds, currentPose, targetPose, currentPoint, segment, currentPointIndex);

    var state = resolveState();

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

  // TODO: We need to decide what state of crossing the bump (if any) we're in via
  // BumpCrossingTracker
  private BumpCrossingState resolveState() {
    return BumpCrossingState.NOT_ON_BUMP;
  }
}
