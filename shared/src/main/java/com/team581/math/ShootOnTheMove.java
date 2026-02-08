package com.team581.math;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class ShootOnTheMove {
  private static final int MAX_ITERATIONS = 5;
  private final InterpolatingDoubleTreeMap distanceToTimeOfFlight;

  public ShootOnTheMove(InterpolatingDoubleTreeMap distanceToTimeOfFlight) {
    this.distanceToTimeOfFlight = distanceToTimeOfFlight;
  }

  public Translation2d getRadialVelocityCompensatedGoal(
      Translation2d robot, Translation2d target, ChassisSpeeds robotVelocity) {
    var result = target;

    for (int i = 0; i < MAX_ITERATIONS; i++) {
      double tof = distanceToTimeOfFlight.get(robot.getDistance(target));

      // 1. Get the direction from robot to target as a Rotation2d
      Rotation2d angleToTarget = target.minus(robot).getAngle();

      // 2. Extract the radial velocity using trigonometry
      double radialVelocityMagnitude =
          (robotVelocity.vxMetersPerSecond * angleToTarget.getCos())
              + (robotVelocity.vyMetersPerSecond * angleToTarget.getSin());

      // 3. Create the offset vector and subtract it
      Translation2d offset = new Translation2d(radialVelocityMagnitude * tof, angleToTarget);
      result = target.minus(offset);
    }
    DogLog.log(
        "ShootOnTheMove/RadialVelocityCompensatedGoal", new Pose2d(result, result.getAngle()));
    return result;
  }

  public Translation2d getTangentialVelocityCompensatedGoal(
      Translation2d robot, Translation2d target, ChassisSpeeds robotVelocity) {
    var result = target;

    for (int i = 0; i < MAX_ITERATIONS; i++) {
      double tof = distanceToTimeOfFlight.get(robot.getDistance(target));

      // 1. Get the radial vector (robot to target)
      Translation2d radialDirection = target.minus(robot).div(robot.getDistance(target));

      // 2. Get the tangential vector (rotate radial by 90 degrees)
      Translation2d tangentialDirection =
          new Translation2d(-radialDirection.getY(), radialDirection.getX());

      // 3. Project robot velocity onto the tangential vector
      double tangentialVelocityMagnitude =
          (robotVelocity.vxMetersPerSecond * tangentialDirection.getX())
              + (robotVelocity.vyMetersPerSecond * tangentialDirection.getY());

      // 4. Offset target sideways
      result = target.minus(tangentialDirection.times(tangentialVelocityMagnitude * tof));
    }
    DogLog.log(
        "ShootOnTheMove/TangentialVelocityCompensatedGoal", new Pose2d(result, result.getAngle()));
    return result;
  }

  public Translation2d getVelocityCompensatedGoal(
      Translation2d robot, Translation2d target, ChassisSpeeds robotVelocity) {
    var timeOfFlight = 0.0;
    var result = target;

    for (int i = 0; i < MAX_ITERATIONS; i++) {
      timeOfFlight = distanceToTimeOfFlight.get(robot.getDistance(result));
      // Compensated goal = real goal - (robot velocity * time of flight of ball)
      result =
          new Translation2d(
              target.getX() - (robotVelocity.vxMetersPerSecond * timeOfFlight),
              target.getY() - (robotVelocity.vyMetersPerSecond * timeOfFlight));
    }

    return result;
  }
}
