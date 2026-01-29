package com.team581.controller;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public final class ControllerHelpers {
  private static final double LOWER_DEADBAND = 0.05;
  private static final double UPPER_DEADBAND = 0.95;

  /**
   * Given the raw X and Y values from a joystick, calculate the magnitude of the vector, apply
   * deadbands, and then apply an exponent.
   *
   * @param joystickX The X value from the joystick.
   * @param joystickY The Y value from the joystick.
   * @param exponent The exponent to apply to the magnitude.
   * @return The magnitude of the vector, with deadbands and exponent applied.
   */
  public static double getJoystickMagnitude(double joystickX, double joystickY, double exponent) {
    var rawMagnitude = Math.hypot(joystickX, joystickY);

    var deadbandedLower = MathUtil.applyDeadband(rawMagnitude, LOWER_DEADBAND, 1);
    var deadbandedUpper =
        MathUtil.interpolate(
            0, Math.signum(deadbandedLower), Math.abs(deadbandedLower) / UPPER_DEADBAND);

    return Math.pow(deadbandedUpper, exponent);
  }

  public static ChassisSpeeds joystickInputsToChassisSpeeds(
      double translationMagnitude,
      Rotation2d direction,
      double rotation,
      double maxSpeed,
      Rotation2d maxAngularRate) {
    var translation = new Translation2d(translationMagnitude, direction);

    var forwardVelocity = translation.getY();
    // For us, negative is left, but WPILib treats positive as left
    var sidewaysVelocity = -translation.getX();

    return new ChassisSpeeds(
        forwardVelocity * maxSpeed,
        sidewaysVelocity * maxSpeed,
        // Robots use CCW+ for rotation, but humans use CW+, so we invert it
        -1.0 * rotation * maxAngularRate.getRadians());
  }

  private ControllerHelpers() {}
}
