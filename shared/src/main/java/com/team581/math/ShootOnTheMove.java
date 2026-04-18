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
  // TODO: Do more thorough tuning with integration test, this was quickly tuned

  public static final DoubleSubscriber DRAG_CONSTANT =
      DogLog.tunable("ShootOnTheMoove/DragCoeff", 0.465);
  private final InterpolatingDoubleTreeMap distanceToTimeOfFlight;

  public ShootOnTheMove(InterpolatingDoubleTreeMap distanceToTimeOfFlight) {
    this.distanceToTimeOfFlight = distanceToTimeOfFlight;
  }

  public record SeparatedVelocityCompensatedGoal(
      double radialVelocity,
      Translation2d radiallyCompensatedGoal,
      double tangentialVelocity,
      Translation2d tangentiallyCompensatedGoal,
      Translation2d fullyCompensatedGoal) {}

  public double getEffectiveTimeOfFlight(double tof) {
    return (1 - Math.pow(Math.E, (-DRAG_CONSTANT.getAsDouble() * tof)))
        / DRAG_CONSTANT.getAsDouble();
  }

  public SeparatedVelocityCompensatedGoal getSeparatedVelocityCompensatedGoal(
      Translation2d robot, Translation2d goal, ChassisSpeeds robotVelocity) {

    double robotX = robot.getX();
    double robotY = robot.getY();
    double goalX = goal.getX();
    double goalY = goal.getY();
    double vx = robotVelocity.vxMetersPerSecond;
    double vy = robotVelocity.vyMetersPerSecond;

    // Rotate the robot velocity vector toward the goal, placing the radial velocity on the x-axis
    // and the tangential velocity on y-axis
    double dx = goalX - robotX;
    double dy = goalY - robotY;
    var robotToGoalAngle = new Rotation2d(dx, dy);
    double cos = robotToGoalAngle.getCos();
    double sin = robotToGoalAngle.getSin();

    // Rotation by -theta:
    // x' = x cos theta + y sin theta
    // y' = -x sin theta + y cos theta
    double radialVelScalar = vx * cos + vy * sin;
    double tangentialVelScalar = -vx * sin + vy * cos;

    double compGoalX = goalX;
    double compGoalY = goalY;
    double timeOfFlight = 0.0;

    // Get time of flight of virtual goal using iterations
    for (int i = 0; i < MAX_ITERATIONS; i++) {
      double dist = Math.hypot(robotX - compGoalX, robotY - compGoalY);
      timeOfFlight = distanceToTimeOfFlight.get(dist);
      // Compensated goal = real goal - (robot velocity * time of flight of ball)
      compGoalX = goalX - (vx * timeOfFlight);
      compGoalY = goalY - (vy * timeOfFlight);
    }

    double radialVelX = radialVelScalar * cos;
    double radialVelY = radialVelScalar * sin;
    double tangentialVelX = -tangentialVelScalar * sin;
    double tangentialVelY = tangentialVelScalar * cos;

    // Apply time of flight to get radially and tangentially compensated goals
    var radiallyCompensatedGoal =
        new Translation2d(goalX - (radialVelX * timeOfFlight), goalY - (radialVelY * timeOfFlight));
    var tangentiallyCompensatedGoal =
        new Translation2d(
            goalX - (tangentialVelX * timeOfFlight), goalY - (tangentialVelY * timeOfFlight));
    var fullyCompensatedGoal = new Translation2d(compGoalX, compGoalY);

    DogLog.log(
        "ShootOnTheMove/CompensatedGoal", new Pose2d(fullyCompensatedGoal, Rotation2d.kZero));

    return new SeparatedVelocityCompensatedGoal(
        radialVelScalar,
        radiallyCompensatedGoal,
        tangentialVelScalar,
        tangentiallyCompensatedGoal,
        fullyCompensatedGoal);
  }

  public SeparatedVelocityCompensatedGoal getSeparatedVelocityCompensatedGoalWithEffectiveTof(
      Translation2d turretTranslation, Translation2d goal, ChassisSpeeds robotVelocity) {

    double turretX = turretTranslation.getX();
    double turretY = turretTranslation.getY();
    double goalX = goal.getX();
    double goalY = goal.getY();
    double vx = robotVelocity.vxMetersPerSecond;
    double vy = robotVelocity.vyMetersPerSecond;

    // Rotate the robot velocity vector toward the goal, placing the radial velocity on the x-axis
    // and the tangential velocity on y-axis
    double dx = goalX - turretX;
    double dy = goalY - turretY;
    var robotToGoalAngle = new Rotation2d(dx, dy);
    double cos = robotToGoalAngle.getCos();
    double sin = robotToGoalAngle.getSin();

    // Rotation by -theta:
    // x' = x cos theta + y sin theta
    // y' = -x sin theta + y cos theta
    double radialVelScalar = vx * cos + vy * sin;
    double tangentialVelScalar = -vx * sin + vy * cos;

    double compGoalX = goalX;
    double compGoalY = goalY;
    double timeOfFlight = 0.0;

    // Get time of flight of virtual goal using iterations
    for (int i = 0; i < MAX_ITERATIONS; i++) {
      double dist = Math.hypot(turretX - compGoalX, turretY - compGoalY);
      timeOfFlight = getEffectiveTimeOfFlight(distanceToTimeOfFlight.get(dist));
      // Compensated goal = real goal - (robot velocity * time of flight of ball)
      compGoalX = goalX - (vx * timeOfFlight);
      compGoalY = goalY - (vy * timeOfFlight);
    }

    double radialVelX = radialVelScalar * cos;
    double radialVelY = radialVelScalar * sin;
    double tangentialVelX = -tangentialVelScalar * sin;
    double tangentialVelY = tangentialVelScalar * cos;

    // Apply time of flight to get radially and tangentially compensated goals
    var radiallyCompensatedGoal =
        new Translation2d(goalX - (radialVelX * timeOfFlight), goalY - (radialVelY * timeOfFlight));
    var tangentiallyCompensatedGoal =
        new Translation2d(
            goalX - (tangentialVelX * timeOfFlight), goalY - (tangentialVelY * timeOfFlight));
    var fullyCompensatedGoal = new Translation2d(compGoalX, compGoalY);

    DogLog.log(
        "ShootOnTheMove/CompensatedGoal", new Pose2d(fullyCompensatedGoal, Rotation2d.kZero));

    return new SeparatedVelocityCompensatedGoal(
        radialVelScalar,
        radiallyCompensatedGoal,
        tangentialVelScalar,
        tangentiallyCompensatedGoal,
        fullyCompensatedGoal);
  }

  public Translation2d getVelocityCompensatedGoal(
      Translation2d robot, Translation2d target, ChassisSpeeds robotVelocity) {
    double robotX = robot.getX();
    double robotY = robot.getY();
    double targetX = target.getX();
    double targetY = target.getY();
    double vx = robotVelocity.vxMetersPerSecond;
    double vy = robotVelocity.vyMetersPerSecond;

    double compGoalX = targetX;
    double compGoalY = targetY;
    double timeOfFlight = 0.0;

    for (int i = 0; i < MAX_ITERATIONS; i++) {
      double dist = Math.hypot(robotX - compGoalX, robotY - compGoalY);
      timeOfFlight = distanceToTimeOfFlight.get(dist);
      // Compensated goal = real goal - (robot velocity * time of flight of ball)
      compGoalX = targetX - (vx * timeOfFlight);
      compGoalY = targetY - (vy * timeOfFlight);
    }

    return new Translation2d(compGoalX, compGoalY);
  }

  public Translation2d getVelocityCompensatedGoalWithEffectiveTof(
      Translation2d robot, Translation2d target, ChassisSpeeds robotVelocity) {
    double robotX = robot.getX();
    double robotY = robot.getY();
    double targetX = target.getX();
    double targetY = target.getY();
    double vx = robotVelocity.vxMetersPerSecond;
    double vy = robotVelocity.vyMetersPerSecond;

    double compGoalX = targetX;
    double compGoalY = targetY;
    double timeOfFlight = 0.0;

    for (int i = 0; i < MAX_ITERATIONS; i++) {
      double dist = Math.hypot(robotX - compGoalX, robotY - compGoalY);
      timeOfFlight = getEffectiveTimeOfFlight(distanceToTimeOfFlight.get(dist));
      // Compensated goal = real goal - (robot velocity * time of flight of ball)
      compGoalX = targetX - (vx * timeOfFlight);
      compGoalY = targetY - (vy * timeOfFlight);
    }

    return new Translation2d(compGoalX, compGoalY);
  }
}
