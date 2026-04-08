package com.team581.trailblazer.followers;

import com.team581.math.MathHelpers;
import com.team581.math.PolarChassisSpeeds;
import com.team581.trailblazer.AngularConstraintOptions;
import com.team581.trailblazer.AutoPoint;
import com.team581.trailblazer.ConstraintsCalculator;
import com.team581.trailblazer.LinearConstraintOptions;
import com.team581.trailblazer.segments.AutoSegment;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class PidPathFollower implements PathFollower {
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
      AutoPoint<?> currentPoint,
      AutoSegment segment,
      int currentPointIndex) {
    // Get constraints for the current point
    var linearConstraints =
        segment.getLinearConstraints(currentPoint).orElseGet(LinearConstraintOptions::new);
    var angularConstraints =
        segment.getAngularConstraints(currentPoint).orElseGet(AngularConstraintOptions::new);

    var distanceToEnd = currentPose.getTranslation().getDistance(targetPose.getTranslation());

    // Sum the total linear distance and total angular work remaining through all waypoints in the
    // segment. These are used together to estimate how long the robot has to complete its rotation,
    // which drives the dynamic angular velocity cap.
    var totalAngularWorkRadians =
        Math.abs(
            MathUtil.angleModulus(
                targetPose.getRotation().getRadians() - currentPose.getRotation().getRadians()));

    // Find total distance and angular work to end of segment
    for (int i = currentPointIndex; i < segment.points.size() - 1; i++) {
      var thisPoint = segment.points.get(i).poseSupplier().get().getPose();
      var nextPoint = segment.points.get(i + 1).getPose();

      distanceToEnd += thisPoint.getTranslation().getDistance(nextPoint.getTranslation());
      totalAngularWorkRadians +=
          Math.abs(
              MathUtil.angleModulus(
                  nextPoint.getRotation().getRadians() - thisPoint.getRotation().getRadians()));
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
            linearVelocity, currentSpeeds, distanceToEnd, linearConstraints);

    // Compute a dynamic angular velocity cap so that rotation is spread evenly across the
    // remaining travel time, rather than rotating as fast as possible and then waiting.
    // estimatedTime = distance / constrained linear velocity gives us roughly how long until we
    // arrive at the end of the segment. neededAngularVelocity = totalAngularWork / estimatedTime
    // gives the steady-state omega to finish all remaining rotation right as we arrive.
    var effectiveAngularConstraints = angularConstraints;

    if (linearVelocity > 1e-3 && distanceToEnd > 1e-3) {
      var estimatedTime = distanceToEnd / linearVelocity;

      if (estimatedTime > 1e-3 && totalAngularWorkRadians > 1e-6) {
        var neededAngularVelocity = totalAngularWorkRadians / estimatedTime;

        DogLog.log("Trailblazer/Follower/TotalAngularWorkRadians", totalAngularWorkRadians);
        DogLog.log("Trailblazer/Follower/DynamicAngularVelocityCap", neededAngularVelocity);

        // Only apply the cap if it is lower than the existing constraint — we don't want to
        // *increase* the allowed velocity, just potentially reduce it to spread rotation out.
        if (neededAngularVelocity < angularConstraints.maxVelocity()) {
          effectiveAngularConstraints =
              new AngularConstraintOptions(
                  neededAngularVelocity, angularConstraints.maxAcceleration());
        }
      }
    }

    angularVelocity =
        velocityConstrainer.constrainAngularVelocity(
            angularVelocity,
            currentPose.getRotation().getRadians(),
            targetPose.getRotation().getRadians(),
            currentSpeeds,
            effectiveAngularConstraints);

    DogLog.log("Trailblazer/Follower/AngularLimit", effectiveAngularConstraints.maxVelocity());
    DogLog.log(
        "Trailblazer/Follower/AngularACCLimit", effectiveAngularConstraints.maxAcceleration());

    DogLog.log("Trailblazer/Follower/AngularVelocityAfterConstraint", angularVelocity);

    var driveDirection = MathHelpers.getDriveDirection(currentPose, targetPose);

    return new PolarChassisSpeeds(linearVelocity, driveDirection, angularVelocity);
  }

  @Override
  public void reset(Pose2d currentPose, ChassisSpeeds currentSpeeds) {
    velocityConstrainer.reset(currentSpeeds, currentPose.getRotation().getRadians());
  }
}
