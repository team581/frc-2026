package com.team581.math;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;

public class BaseTurretCalculator {

  public static double calculateHomedPositionFromMotorAndEncoder(
      double turretMotorPosition,
      double turretEncoderPosition,
      double motorToTurretRatio,
      double encoderToTurretRatio,
      double motorRotationResolution,
      double rotorCalibratedOffset) {
    double rotor_position = (turretMotorPosition % 1) - rotorCalibratedOffset;
    DogLog.log("Turret/Calculator/motor_mod", rotor_position);
    double rotorRotationsRelativeToTurret = rotor_position / motorToTurretRatio;
    double roughAbsolutePosition = turretEncoderPosition / encoderToTurretRatio;
    DogLog.log("Turret/Calculator/rough_abs_pos", roughAbsolutePosition);

    int potentialMotorWrapA =
        (int) (roughAbsolutePosition / motorRotationResolution); // motor_rotation_resolution;
    DogLog.log("Turret/Calculator/potentialA", potentialMotorWrapA);

    double potentialMotorWrapB = potentialMotorWrapA - 1;
    double potentialMotorWrapC = potentialMotorWrapA + 1;

    double potentialMotorPosA =
        (potentialMotorWrapA * motorRotationResolution) + rotorRotationsRelativeToTurret;
    double potentialMotorPosB =
        (potentialMotorWrapB * motorRotationResolution) + rotorRotationsRelativeToTurret;
    double potentialMotorPosC =
        (potentialMotorWrapC * motorRotationResolution) + rotorRotationsRelativeToTurret;

    double potentialMotorPosErrA = Math.abs(roughAbsolutePosition - potentialMotorPosA);
    double potentialMotorPositionErrB = Math.abs(roughAbsolutePosition - potentialMotorPosB);
    double potentialMotorPosErrC = Math.abs(roughAbsolutePosition - potentialMotorPosC);

    double turretPos = potentialMotorPosC;
    if (potentialMotorPosErrA < potentialMotorPositionErrB
        && potentialMotorPosErrA < potentialMotorPosErrC) {
      turretPos = potentialMotorPosA;
    }
    if (potentialMotorPositionErrB < potentialMotorPosErrA
        && potentialMotorPositionErrB < potentialMotorPosErrC) {
      turretPos = potentialMotorPosB;
    }
    DogLog.log("Turret/Calculator/turretPos", turretPos);
    DogLog.log("Turret/Calculator/turretPosDegrees", Units.rotationsToDegrees(turretPos));

    return turretPos;
  }

  public static double calculateSwerveTurretCompensationAngle(
      double wantedTurretAngle,
      Rotation2d robotRotation,
      double minTurretAngle,
      double maxTurretAngle) {
    var robotHeading = robotRotation.getDegrees();
    robotHeading = MathHelpers.angleModulus(robotHeading);
    wantedTurretAngle = MathHelpers.angleModulus(wantedTurretAngle);
    var actualTargetRotation = MathHelpers.angleModulus(wantedTurretAngle + robotHeading);
    if (wantedTurretAngle > 0.0) {

      return MathHelpers.angleModulus(actualTargetRotation + 60.0) - minTurretAngle;
    } else {
      return MathHelpers.angleModulus(actualTargetRotation - 60.0) - maxTurretAngle;
    }
  }

  public static double calculateTurretAimingAngle(
      Pose2d robot, Translation2d target, Transform2d turretToRobot) {
    var fieldRelativeTurretPose =
        robot.getTranslation().plus(turretToRobot.getTranslation().rotateBy(robot.getRotation()));
    var targetAngle =
        Math.toDegrees(
            Math.atan2(
                target.getY() - fieldRelativeTurretPose.getY(),
                target.getX() - fieldRelativeTurretPose.getX()));
    var robotHeading = robot.getRotation().getDegrees();

    targetAngle = MathHelpers.angleModulus(targetAngle);
    robotHeading = MathHelpers.angleModulus(robotHeading);

    return MathHelpers.angleModulus(targetAngle - robotHeading);
  }

  public static boolean doesTurretHaveRoom(
      double turretAngle,
      double minTurretAngle,
      double maxTurretAngle,
      double wantedSpaceFromHardstop,
      double spaceFromHardstopTolerance) {
    if (turretAngle > 0) {
      if (turretAngle < maxTurretAngle - wantedSpaceFromHardstop + spaceFromHardstopTolerance) {
        return true;
      }
    }
    if (turretAngle < 0) {
      if (turretAngle > minTurretAngle + wantedSpaceFromHardstop - spaceFromHardstopTolerance) {
        return true;
      }
    }
    return false;
  }

  public static double getGoalCentricTurretTolerance(
      Translation2d goalTranslation,
      Pose2d robotPose,
      double goalCentricToleranceMeters,
      Transform2d turretToRobot) {
    var fieldRelativeTurretPose =
        robotPose
            .getTranslation()
            .plus(turretToRobot.getTranslation().rotateBy(robotPose.getRotation()));
    double distanceToGoal = fieldRelativeTurretPose.getDistance(goalTranslation);
    return Math.toDegrees(Math.atan2(goalCentricToleranceMeters, distanceToGoal));
  }

  public static double getOptimalAngle(
      double target, double current, double minTurretAngle, double maxTurretAngle) {
    target = MathHelpers.angleModulus(target);

    // Get smallest delta
    var delta = ((target - current + 180) % 360 + 360) % 360 - 180;

    var option1 = current + delta;

    var option2 = (option1 > 0) ? option1 - 360 : option1 + 360;

    var opt1Valid = (option1 >= minTurretAngle && option1 <= maxTurretAngle);
    var opt2Valid = (option2 >= minTurretAngle && option2 <= maxTurretAngle);

    if (opt1Valid && opt2Valid) {

      // If both are reachable, pick the with the least movement
      return MathUtil.clamp(
          (Math.abs(option1 - current) <= Math.abs(option2 - current)) ? option1 : option2,
          minTurretAngle,
          maxTurretAngle);
    }

    if (opt1Valid) {

      return MathUtil.clamp(option1, minTurretAngle, maxTurretAngle);
    }

    if (opt2Valid) {

      return MathUtil.clamp(option2, minTurretAngle, maxTurretAngle);
    }

    return MathUtil.clamp(option1, minTurretAngle, maxTurretAngle);
  }

  // Look at UnwrapAngleDiagram.png
  public static double getSmartUnwrapAngle(
      double target,
      double current,
      double minTurretAngle,
      double maxTurretAngle,
      double tolerance) {

    target = getOptimalAngle(target, current, minTurretAngle, maxTurretAngle);
    var totalRangeOfMotion = Math.abs(maxTurretAngle) + Math.abs(minTurretAngle);
    if (totalRangeOfMotion <= 360) {
      return target;
    }

    // Make sure that no angle can be equally in both ends' bad range
    var maxTolerance = (totalRangeOfMotion - 360) / 4;
    tolerance = MathUtil.clamp(tolerance, 0, maxTolerance);
    if (target >= (maxTurretAngle - tolerance) && target <= maxTurretAngle) {
      return MathUtil.clamp(target - 360, minTurretAngle, maxTurretAngle);
    }

    if (target >= minTurretAngle && target <= (minTurretAngle + tolerance)) {

      return MathUtil.clamp(target + 360, minTurretAngle, maxTurretAngle);
    }

    return target;
  }

  public static ChassisSpeeds getTurretChassisSpeeds(
      ChassisSpeeds robotSpeeds, double robotHeading, Translation2d turretToRobot) {
    var angularVelocity = robotSpeeds.omegaRadiansPerSecond;
    Translation2d fieldRelativeOffset =
        turretToRobot.rotateBy(Rotation2d.fromDegrees(robotHeading));
    var turretSwingX = -angularVelocity * fieldRelativeOffset.getY();
    var turretSwingY = angularVelocity * fieldRelativeOffset.getX();
    var turretTotalVelocityX = robotSpeeds.vxMetersPerSecond + turretSwingX;
    var turretTotalVelocityY = robotSpeeds.vyMetersPerSecond + turretSwingY;
    return new ChassisSpeeds(turretTotalVelocityX, turretTotalVelocityY, angularVelocity);
  }

  public static Pose2d getTurretPose(Pose2d robot, Transform2d turretToRobot) {
    return robot.plus(turretToRobot);
  }
}
