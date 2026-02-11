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
    return result;
  }

  // TODO: Make this function able to return a tangential and radial goal
  public void getVelocityGoalCompensation(
      Translation2d robot, Translation2d target, ChassisSpeeds robotVelocity) {
      var radialCompensatedGoal = target;
      var tangentialCompensatedGoal = target;

      var robotToTargetTranslation = new Translation2d(target.getX() - robot.getX(), target.getY() - robot.getY());
      // Rotate robot velocity vector toward the target, placing the radial velocity on x axis and the tangential velocity on y axis
      var velocityToTarget = new Translation2d(robotVelocity.vxMetersPerSecond, robotVelocity.vyMetersPerSecond).rotateBy(robotToTargetTranslation.getAngle());
      // Take only the radial/tangential velocity in the velocity to target vector, then convert it to field relative by rotating it back
      var radialVelocity = new Translation2d(velocityToTarget.getX(), 0.0).rotateBy(robotToTargetTranslation.getAngle().times(-1.0));
      var tangentialVelocity = new Translation2d(0.0, velocityToTarget.getY()).rotateBy(robotToTargetTranslation.getAngle().times(-1.0));

      var radialTOF = 0.0;
      var tangentialTOF = 0.0;

      for (int i = 0; i < MAX_ITERATIONS; i++) {
        radialTOF = distanceToTimeOfFlight.get(robot.getDistance(radialCompensatedGoal));
        tangentialTOF = distanceToTimeOfFlight.get(robot.getDistance(tangentialCompensatedGoal));

        // Compensated goal = real goal - (robot velocity * time of flight of ball)
        radialCompensatedGoal =
        new Translation2d(
          radialCompensatedGoal.getX() - (radialVelocity.getX() * radialTOF),
          radialCompensatedGoal.getY() - (radialVelocity.getY() * radialTOF));

          tangentialCompensatedGoal =
          new Translation2d(
            tangentialCompensatedGoal.getX() - (tangentialVelocity.getX() * tangentialTOF),
            tangentialCompensatedGoal.getY() - (tangentialVelocity.getY() * tangentialTOF));
      }

      DogLog.log("ShootOnTheMove/RadialCompensatedGoal", new Pose2d(radialCompensatedGoal, Rotation2d.kZero));
      DogLog.log("ShootOnTheMove/TangentialCompensatedGoal", new Pose2d(tangentialCompensatedGoal, Rotation2d.kZero));

    // return ;
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

    DogLog.log("ShootOnTheMove/CompensatedGoal", new Pose2d(result, Rotation2d.kZero));

    return result;
  }
}
