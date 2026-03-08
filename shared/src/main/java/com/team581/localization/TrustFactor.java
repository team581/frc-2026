package com.team581.localization;

import com.team581.vision.results.TagResult;
import dev.doglog.DogLog;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.List;

public class TrustFactor {
  private static final DoubleSubscriber POST_COLLISION_ADDITION =
      DogLog.tunable("TrustFactor/PostCollisionAddition", 2.0);
  private static final DoubleSubscriber TRUSTWORTHY_THRESHOLD =
      DogLog.tunable("TrustFactor/TrustworthyThreshold", Units.inchesToMeters(5));

  private static final DoubleSubscriber LOST_TOLERANCE =
      DogLog.tunable("TrustFactor/LostThreshold", 1.0);
  private double trustFactor = Double.POSITIVE_INFINITY;
  private double metersTravelledSinceLastCheck = 0.0;
  private Pose2d lastCheckedPose = Pose2d.kZero;

  public double get() {
    return trustFactor;
  }

  public void ingestTagResult(Pose2d globalPose, List<TagResult> results) {
    if (results.isEmpty()) {
      return;
    }

    var totalStdX = 0.0;
    var totalStdY = 0.0;

    for (TagResult result : results) {
      Vector<N3> stdDevs = result.standardDevs();
      totalStdX += stdDevs.get(0, 0);
      totalStdY += stdDevs.get(1, 0);
    }

    if (totalStdX <= 0 || totalStdY <= 0) {
      return;
    }

    var avgX = 0.0;
    var avgY = 0.0;
    var weightedStdX = 0.0;
    var weightedStdY = 0.0;

    for (TagResult result : results) {
      var pose = result.pose();
      var stdDevs = result.standardDevs();
      var stdX = stdDevs.get(0, 0);
      var stdY = stdDevs.get(1, 0);

      // weight = 1 - (std / total_std)
      var weightX = (stdX / totalStdX);
      var weightY = (stdY / totalStdY);

      // add (weight * pose) to the total
      avgX += weightX * pose.getX();
      avgY += weightY * pose.getY();

      // Add (weight * std) to the total
      weightedStdX += weightX * stdX;
      weightedStdY += weightY * stdY;
    }

    var avgVisionPose = new Pose2d(avgX, avgY, Rotation2d.kZero);

    // same as (global_pose - avg_vision)
    double poseDifference = globalPose.getTranslation().getDistance(avgVisionPose.getTranslation());

    trustFactor = poseDifference + weightedStdX + weightedStdY;
  }

  public boolean isLost() {
    // Bypass trust factor checks in simulation, since we don't have simulated cameras
    if (RobotBase.isSimulation()) {
      return false;
    }

    return trustFactor >= LOST_TOLERANCE.get();
  }

  public boolean isTrustworthy() {
    // Bypass trust factor checks in simulation, since we don't have simulated cameras
    if (RobotBase.isSimulation()) {
      return true;
    }

    return trustFactor <= TRUSTWORTHY_THRESHOLD.get();
  }

  public void reset() {
    trustFactor += LOST_TOLERANCE.get();
  }

  public void seededPose() {
    trustFactor = TRUSTWORTHY_THRESHOLD.get();
  }

  public void update(
      Pose2d robotPose, double odometryStandardDeviation, boolean collisionDetected) {
    metersTravelledSinceLastCheck =
        lastCheckedPose.getTranslation().getDistance(robotPose.getTranslation());

    trustFactor += metersTravelledSinceLastCheck * odometryStandardDeviation;
    lastCheckedPose = robotPose;

    if (collisionDetected) {
      trustFactor += POST_COLLISION_ADDITION.get();
    }
  }
}
