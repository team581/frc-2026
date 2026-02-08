package com.team581.trailblazer.followers;

import com.team581.math.MathHelpers;
import com.team581.math.PolarChassisSpeeds;
import com.team581.trailblazer.AutoConstraintOptions;
import com.team581.trailblazer.AutoPoint;
import com.team581.trailblazer.ConstraintsCalculator;
import com.team581.trailblazer.segments.AutoSegment;
import dev.doglog.DogLog;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class PidPathFollower implements PathFollower {
  // 0-1 scalar
  private static final double AGGRESSIVENESS_FACTOR = 0.5;
  private final PIDController translationController;
  private final PIDController rotationController;
  private final ConstraintsCalculator velocityConstrainer;

  public PidPathFollower(PIDController translationController, PIDController rotationController) {
    this.translationController = translationController;
    this.rotationController = rotationController;
    this.velocityConstrainer = new ConstraintsCalculator(rotationController);

    this.rotationController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public ChassisSpeeds calculateSpeeds(
      ChassisSpeeds currentSpeeds,
      Pose2d currentPose,
      Pose2d targetPose,
      AutoPoint currentPoint,
      AutoSegment segment,
      int currentPointIndex) {
    // Get constraints for the current point
    var constraints = segment.getConstraints(currentPoint).orElseGet(AutoConstraintOptions::new);

    var distanceToEnd = currentPose.getTranslation().getDistance(targetPose.getTranslation());

    // Find total distance to end of segment
    for (int i = currentPointIndex; i < segment.points.size() - 1; i++) {
      distanceToEnd +=
          segment
              .points
              .get(i)
              .poseSupplier()
              .get()
              .getPose()
              .getTranslation()
              .getDistance(
                  segment.points.get(i + 1).poseSupplier().get().getPose().getTranslation());
    }

    // Calculate velocities with PID controller
    var linearVelocity = Math.abs(translationController.calculate(distanceToEnd, 0));
    var angularVelocity =
        rotationController.calculate(
            currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

    DogLog.log("Trailblazer/Follower/AngularVelocity", angularVelocity);

    // Constrain velocities
    linearVelocity =
        velocityConstrainer.constrainLinearVelocity(
            linearVelocity, currentSpeeds, distanceToEnd, constraints);
    angularVelocity =
        velocityConstrainer.constrainAngularVelocity(
            angularVelocity,
            currentPose.getRotation().getRadians(),
            targetPose.getRotation().getRadians(),
            currentSpeeds,
            constraints);

    DogLog.log("Trailblazer/Follower/AngularLimit", constraints.maxAngularVelocity());
    DogLog.log("Trailblazer/Follower/AngularACCLimit", constraints.maxAngularAcceleration());

    DogLog.log("Trailblazer/Follower/AngularVelocityAfterConstraint", angularVelocity);

    var driveDirection = MathHelpers.getDriveDirection(currentPose, targetPose);

    if (MathHelpers.getLinearVelocity(currentSpeeds) > 0.2 && distanceToEnd > 0.1) {
      var currentDirection = MathHelpers.getDriveDirection(currentSpeeds);
      driveDirection = currentDirection.interpolate(driveDirection, AGGRESSIVENESS_FACTOR);
    }

    return new PolarChassisSpeeds(linearVelocity, driveDirection, angularVelocity);
  }

  @Override
  public void reset(ChassisSpeeds currentSpeeds, double currentAngleRadians) {
    velocityConstrainer.reset(currentSpeeds, currentAngleRadians);
  }
}
