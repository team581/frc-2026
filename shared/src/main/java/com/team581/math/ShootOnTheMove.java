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

  public record SeparatedVelocityCompensatedGoal(
      Translation2d radiallyCompensatedGoal, Translation2d tangentiallyCompensatedGoal) {}

  public SeparatedVelocityCompensatedGoal getSeparatedVelocityCompensatedGoal(
      Translation2d robot, Translation2d goal, ChassisSpeeds robotVelocity) {
    var radiallyCompensatedGoal = goal;
    var tangentiallyCompensatedGoal = goal;
    var compensatedGoal = goal;
    var timeOfFlight = 0.0;

    // Rotate the robot velocity vector toward the goal, placing the radial velocity on the x-axis
    // and the tangential velocity on y-axis
    var robotToGoalAngle = goal.minus(robot).getAngle();
    var velocityTowardGoal =
        new Translation2d(robotVelocity.vxMetersPerSecond, robotVelocity.vyMetersPerSecond)
            .rotateBy(robotToGoalAngle.times(-1.0));
    // Take only the radial/tangential velocity then rotate it back to field relative
    var radialVelocity =
        new Translation2d(velocityTowardGoal.getX(), 0.0).rotateBy(robotToGoalAngle);
    var tangentialVelocity =
        new Translation2d(0.0, velocityTowardGoal.getY()).rotateBy(robotToGoalAngle);

    for (int i = 0; i < MAX_ITERATIONS; i++) {
      timeOfFlight = distanceToTimeOfFlight.get(robot.getDistance(compensatedGoal));
      // Compensated goal = real goal - (robot velocity * time of flight of ball)

      compensatedGoal =
          new Translation2d(
              goal.getX() - (robotVelocity.vxMetersPerSecond * timeOfFlight),
              goal.getY() - (robotVelocity.vyMetersPerSecond * timeOfFlight));
      radiallyCompensatedGoal =
          new Translation2d(
              goal.getX() - (radialVelocity.getX() * timeOfFlight),
              goal.getY() - (radialVelocity.getY() * timeOfFlight));
      tangentiallyCompensatedGoal =
          new Translation2d(
              goal.getX() - (tangentialVelocity.getX() * timeOfFlight),
              goal.getY() - (tangentialVelocity.getY() * timeOfFlight));
    }

    DogLog.log("ShootOnTheMove/CompensatedGoal", new Pose2d(compensatedGoal, Rotation2d.kZero));

    return new SeparatedVelocityCompensatedGoal(
        radiallyCompensatedGoal, tangentiallyCompensatedGoal);
  }

  /**
   * @deprecated Use {@link #getSeparatedVelocityCompensatedGoal(Translation2d, Translation2d,
   *     ChassisSpeeds)}
   */
  @Deprecated
  public Translation2d getVelocityCompensatedGoal(
      Translation2d robot, Translation2d goal, ChassisSpeeds robotVelocity) {
    var timeOfFlight = 0.0;
    var result = goal;

    for (int i = 0; i < MAX_ITERATIONS; i++) {
      timeOfFlight = distanceToTimeOfFlight.get(robot.getDistance(result));
      // Compensated goal = real goal - (robot velocity * time of flight of ball)
      result =
          new Translation2d(
              goal.getX() - (robotVelocity.vxMetersPerSecond * timeOfFlight),
              goal.getY() - (robotVelocity.vyMetersPerSecond * timeOfFlight));
    }

    DogLog.log("ShootOnTheMove/CompensatedGoal", new Pose2d(result, Rotation2d.kZero));

    return result;
  }
}
