package com.team581.trailblazer.followers;

import com.team581.math.MathHelpers;
import com.team581.math.PolarChassisSpeeds;
import com.team581.trailblazer.AutoConstraintOptions;
import com.team581.trailblazer.AutoPoint;
import com.team581.trailblazer.AutoSegment;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class PidPathFollower implements PathFollower {
  private final PIDController translationController;
  private final PIDController rotationController;

  public PidPathFollower(PIDController translationController, PIDController rotationController) {
    this.translationController = translationController;
    this.rotationController = rotationController;

    rotationController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public ChassisSpeeds calculateSpeeds(
      Pose2d currentPose,
      Pose2d targetPose,
      AutoPoint currentPoint,
      AutoSegment segment,
      int currentPointIndex) {
    // Get constraints for the current point
    var constraints = segment.getConstraints(currentPoint).orElseGet(AutoConstraintOptions::new);

    // Calculate distance to goal
    var distanceToGoalMeters =
        currentPose.getTranslation().getDistance(targetPose.getTranslation());

    var driveVelocity = Math.abs(translationController.calculate(distanceToGoalMeters, 0));

    var angularVelocity =
        rotationController.calculate(
            currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

    // Calculate drive direction
    var driveDirection = MathHelpers.getDriveDirection(currentPose, targetPose);

    // Apply constraints - BASIC PLACEHOLDER IMPLEMENTATION
    // Cap max linear velocity based on constraints
    if (constraints.maxLinearVelocity() > 0) {
      driveVelocity = Math.min(driveVelocity, constraints.maxLinearVelocity());
    }

    // Cap max angular velocity based on constraints
    if (constraints.maxAngularVelocity() > 0) {
      angularVelocity =
          MathUtil.clamp(
              angularVelocity, -constraints.maxAngularVelocity(), constraints.maxAngularVelocity());
    }

    // TODO: Add angle bisector

    return new PolarChassisSpeeds(driveVelocity, driveDirection, angularVelocity);
  }
}
