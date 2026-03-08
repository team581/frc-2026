package com.team581.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class DriveConstraints {

  /**
   * Scales the driver's requested field-relative speeds to respect radial and tangential limits
   *
   * @param requestedSpeeds Field-relative requested speeds
   * @param robotTranslation Current field-relative position of the robot
   * @param goalTranslation Field-relative position of the goal
   * @return A scaled requested chassis speeds
   */
  public static ChassisSpeeds getGoalCentricConstrainedSpeeds(
      ChassisSpeeds requestedSpeeds,
      Translation2d robotTranslation,
      Translation2d goalTranslation,
      double maxRadialVelocity,
      double maxTangentialVelocity) {

    Rotation2d angleToGoal = goalTranslation.minus(robotTranslation).getAngle();

    Translation2d requestedVelocity =
        new Translation2d(requestedSpeeds.vxMetersPerSecond, requestedSpeeds.vyMetersPerSecond);

    // X becomes radial velocity (towards/away), Y becomes tangential (strafing)
    Translation2d goalCentricVelocity = requestedVelocity.rotateBy(angleToGoal.unaryMinus());

    // Calculate how much we are exceeding the radial limit
    double radialScale = 1.0;
    if (Math.abs(goalCentricVelocity.getX()) > maxRadialVelocity) {
      radialScale = maxRadialVelocity / Math.abs(goalCentricVelocity.getX());
    }

    // Calculate how much we are exceeding the tangential limit
    double tangentialScale = 1.0;
    if (Math.abs(goalCentricVelocity.getY()) > maxTangentialVelocity) {
      tangentialScale = maxTangentialVelocity / Math.abs(goalCentricVelocity.getY());
    }

    // Find the most restrictive scale factor
    double desaturationFactor = Math.min(radialScale, tangentialScale);

    // Scale the requested velocity by the desaturation factor
    Translation2d safeVelocity = requestedVelocity.times(desaturationFactor);

    return new ChassisSpeeds(
        safeVelocity.getX(), safeVelocity.getY(), requestedSpeeds.omegaRadiansPerSecond);
  }
}
