package com.team581.localization;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.RobotBase;

public class TrustFactor {
  private static final DoubleSubscriber DISTANCE_TRAVELLED_SCALAR =
      DogLog.tunable("TrustFactor/DistanceTravelledScalar", 1.0);
  private static final DoubleSubscriber POST_COLLISION_ADDITION =
      DogLog.tunable("TrustFactor/PostCollisionAddition", 5.0);
  private static final DoubleSubscriber TRUSTWORTHY_THRESHOLD =
      DogLog.tunable("TrustFactor/TrustworthyThreshold", 1.0);
  private static final DoubleSubscriber TAG_SEEN_DENOMINATOR =
      DogLog.tunable("TrustFactor/TagSeenDenominator", 3.0);
  private static final DoubleSubscriber TAG_SEEN_MAX =
      DogLog.tunable("TrustFactor/TagSeenMax", 5.0);
  private static final DoubleSubscriber LOST_THRESHOLD =
      DogLog.tunable("TrustFactor/LostThreshold", 10.0);
  private double trustFactor = Double.POSITIVE_INFINITY;
  private double metersTravelledSinceLastCheck = 0.0;
  private Pose2d lastCheckedPose = Pose2d.kZero;

  public double get() {
    return trustFactor;
  }

  public boolean isLost() {
    // Bypass trust factor checks in simulation, since we don't have simulated cameras
    if (RobotBase.isSimulation()) {
      return false;
    }

    return trustFactor >= LOST_THRESHOLD.get();
  }

  public boolean isTrustworthy() {
    // Bypass trust factor checks in simulation, since we don't have simulated cameras
    if (RobotBase.isSimulation()) {
      return true;
    }

    return trustFactor <= TRUSTWORTHY_THRESHOLD.get();
  }

  public void reset() {
    trustFactor += LOST_THRESHOLD.get();
  }

  public void seededPose() {
    trustFactor = TRUSTWORTHY_THRESHOLD.get();
  }

  public void tagSeen() {
    trustFactor = Math.min(trustFactor / TAG_SEEN_DENOMINATOR.get(), TAG_SEEN_MAX.get());
  }

  public void update(Pose2d robotPose, boolean collisionDetected) {
    metersTravelledSinceLastCheck =
        lastCheckedPose.getTranslation().getDistance(robotPose.getTranslation());

    trustFactor += metersTravelledSinceLastCheck * DISTANCE_TRAVELLED_SCALAR.get();
    lastCheckedPose = robotPose;

    if (collisionDetected) {
      trustFactor += POST_COLLISION_ADDITION.get();
    }
  }
}
