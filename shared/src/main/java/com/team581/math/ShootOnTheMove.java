package com.team581.math;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class ShootOnTheMove {
  private static final int MAX_ITERATIONS = 5;
  // TODO: find drag constant. Currently eyeballed
  // https://frc-docs--3242.org.readthedocs.build/en/3242/docs/software/advanced-controls/fire-control/linear-drag.html
  public static final double DRAG_CONSTANT = 0.9523;
  private final InterpolatingDoubleTreeMap distanceToTimeOfFlight;

  public ShootOnTheMove(InterpolatingDoubleTreeMap distanceToTimeOfFlight) {
    this.distanceToTimeOfFlight = distanceToTimeOfFlight;
  }

  public double getEffectiveTimeOfFlight(double tof) {
    return (1 - Math.pow(Math.E, (-ShootOnTheMove.DRAG_CONSTANT * tof)))
        / ShootOnTheMove.DRAG_CONSTANT;
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
