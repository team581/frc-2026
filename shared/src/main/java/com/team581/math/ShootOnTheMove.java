package com.team581.math;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoubleSubscriber;

public class ShootOnTheMove {
  private static final int MAX_ITERATIONS = 5;
  // TODO: find drag constant. Currently eyeballed
  // https://frc-docs--3242.org.readthedocs.build/en/3242/docs/software/advanced-controls/fire-control/linear-drag.html
  public static final DoubleSubscriber DRAG_CONSTANT =
      DogLog.tunable("ShootOnTheMoove/DragCoeff", 0.5);
  private final InterpolatingDoubleTreeMap distanceToTimeOfFlight;

  public ShootOnTheMove(InterpolatingDoubleTreeMap distanceToTimeOfFlight) {
    this.distanceToTimeOfFlight = distanceToTimeOfFlight;
  }

  public record SeparatedVelocityCompensatedGoal(
      Translation2d radiallyCompensatedGoal, Translation2d tangentiallyCompensatedGoal) {}

  public double getEffectiveTimeOfFlight(double tof) {
    return (1 - Math.pow(Math.E, (-DRAG_CONSTANT.getAsDouble() * tof)))
        / DRAG_CONSTANT.getAsDouble();
  }

  public SeparatedVelocityCompensatedGoal getSeparatedVelocityCompensatedGoal(
      Translation2d robot, Translation2d goal, ChassisSpeeds robotVelocity) {

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

    // Get time of flight of virtual goal using iterations
    for (int i = 0; i < MAX_ITERATIONS; i++) {
      timeOfFlight = distanceToTimeOfFlight.get(robot.getDistance(compensatedGoal));
      // Compensated goal = real goal - (robot velocity * time of flight of ball)
      compensatedGoal =
          new Translation2d(
              goal.getX() - (robotVelocity.vxMetersPerSecond * timeOfFlight),
              goal.getY() - (robotVelocity.vyMetersPerSecond * timeOfFlight));
    }

    // Apply time of flight to get radially and tangentially compensated goals
    var radiallyCompensatedGoal =
        new Translation2d(
            goal.getX() - (radialVelocity.getX() * timeOfFlight),
            goal.getY() - (radialVelocity.getY() * timeOfFlight));
    var tangentiallyCompensatedGoal =
        new Translation2d(
            goal.getX() - (tangentialVelocity.getX() * timeOfFlight),
            goal.getY() - (tangentialVelocity.getY() * timeOfFlight));

    DogLog.log("ShootOnTheMove/CompensatedGoal", new Pose2d(compensatedGoal, Rotation2d.kZero));

    return new SeparatedVelocityCompensatedGoal(
        radiallyCompensatedGoal, tangentiallyCompensatedGoal);
  }

  public SeparatedVelocityCompensatedGoal getSeparatedVelocityCompensatedGoalWithEffectiveTof(
      Translation2d robot, Translation2d goal, ChassisSpeeds robotVelocity) {

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

    // Get time of flight of virtual goal using iterations
    for (int i = 0; i < MAX_ITERATIONS; i++) {
      timeOfFlight =
          getEffectiveTimeOfFlight(distanceToTimeOfFlight.get(robot.getDistance(compensatedGoal)));
      // Compensated goal = real goal - (robot velocity * time of flight of ball)
      compensatedGoal =
          new Translation2d(
              goal.getX() - (robotVelocity.vxMetersPerSecond * timeOfFlight),
              goal.getY() - (robotVelocity.vyMetersPerSecond * timeOfFlight));
    }

    // Apply time of flight to get radially and tangentially compensated goals
    var radiallyCompensatedGoal =
        new Translation2d(
            goal.getX() - (radialVelocity.getX() * timeOfFlight),
            goal.getY() - (radialVelocity.getY() * timeOfFlight));
    var tangentiallyCompensatedGoal =
        new Translation2d(
            goal.getX() - (tangentialVelocity.getX() * timeOfFlight),
            goal.getY() - (tangentialVelocity.getY() * timeOfFlight));

    DogLog.log("ShootOnTheMove/CompensatedGoal", new Pose2d(compensatedGoal, Rotation2d.kZero));

    return new SeparatedVelocityCompensatedGoal(
        radiallyCompensatedGoal, tangentiallyCompensatedGoal);
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

  public Translation2d getVelocityCompensatedGoalWithEffectiveTof(
      Translation2d robot, Translation2d target, ChassisSpeeds robotVelocity) {
    var timeOfFlight = 0.0;
    var effectiveTimeOfFlight = 0.0;
    var result = target;

    for (int i = 0; i < MAX_ITERATIONS; i++) {
      timeOfFlight = distanceToTimeOfFlight.get(robot.getDistance(result));
      effectiveTimeOfFlight = getEffectiveTimeOfFlight(timeOfFlight);
      // Compensated goal = real goal - (robot velocity * time of flight of ball)
      result =
          new Translation2d(
              target.getX() - (robotVelocity.vxMetersPerSecond * effectiveTimeOfFlight),
              target.getY() - (robotVelocity.vyMetersPerSecond * effectiveTimeOfFlight));
    }

    return result;
  }
}
