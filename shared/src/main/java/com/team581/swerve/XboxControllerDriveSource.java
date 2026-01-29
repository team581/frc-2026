package com.team581.swerve;

import com.team581.controller.ControllerHelpers;
import com.team581.math.MathHelpers;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;

public class XboxControllerDriveSource implements DriveSource {
  private final XboxController controller;
  private final double maxLinearVelocity;
  private final Rotation2d maxAngularVelocity;

  public XboxControllerDriveSource(
      XboxController controller, double maxLinearVelocity, Rotation2d maxAngularVelocity) {
    this.controller = controller;
    this.maxLinearVelocity = maxLinearVelocity;
    this.maxAngularVelocity = maxAngularVelocity;
  }

  @Override
  public DriveSourceType getDriveSourceType() {
    return DriveSourceType.DRIVER_PERSPECTIVE_OPEN_LOOP;
  }

  @Override
  public ChassisSpeeds getRequestedSpeeds() {
    var leftX = controller.getLeftX();
    var leftY = -controller.getLeftY();
    var rightX = controller.getRightX();

    var translationMagnitude = ControllerHelpers.getJoystickMagnitude(leftX, leftY, 2);
    var rotationMagnitude =
        Math.copySign(ControllerHelpers.getJoystickMagnitude(rightX, 0, 5), rightX);

    return ControllerHelpers.joystickInputsToChassisSpeeds(
        translationMagnitude,
        MathHelpers.rotation2d(leftX, leftY),
        rotationMagnitude,
        maxLinearVelocity,
        maxAngularVelocity);
  }
}
