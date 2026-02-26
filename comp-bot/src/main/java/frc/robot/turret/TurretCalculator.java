package frc.robot.turret;

import com.team581.math.BaseTurretCalculator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class TurretCalculator {

  private static final double SPACE_FROM_HARDSTOP = 0.0;
  private static final double SPACE_FROM_HARDSTOP_TOLERANCE = 0.0;

  public static double calculateHomedPositionFromMotorAndEncoder(
      double turretMotorPosition, double turretEncoderPosition) {
    return BaseTurretCalculator.calculateHomedPositionFromMotorAndEncoder(
        turretMotorPosition,
        turretEncoderPosition,
        TurretConfig.MOTOR_TO_TURRET,
        TurretConfig.ENCODER_TO_TURRET,
        TurretConfig.MOTOR_ROTATION_RESOLUTION);
  }

  public static double calculateSwerveTurretCompensationAngle(
      double wantedTurretAngle, Rotation2d robotRotation) {
    return BaseTurretCalculator.calculateSwerveTurretCompensationAngle(
        wantedTurretAngle, robotRotation, TurretConfig.MIN_ANGLE, TurretConfig.MAX_ANGLE);
  }

  public static double calculateTurretAimingAngle(Pose2d robot, Translation2d target) {
    return BaseTurretCalculator.calculateTurretAimingAngle(
        robot, target, TurretConfig.TURRET_TO_ROBOT);
  }

  public static boolean doesTurretHaveRoom(double turretAngle) {
    return BaseTurretCalculator.doesTurretHaveRoom(
        turretAngle,
        TurretConfig.MIN_ANGLE,
        TurretConfig.MAX_ANGLE,
        SPACE_FROM_HARDSTOP,
        SPACE_FROM_HARDSTOP_TOLERANCE);
  }

  public static double getOptimalAngle(double target, double current) {
    return BaseTurretCalculator.getOptimalAngle(
        target, current, TurretConfig.MIN_ANGLE, TurretConfig.MAX_ANGLE);
  }

  public static double getSmartUnwrapAngle(double target, double current) {
    return BaseTurretCalculator.getSmartUnwrapAngle(
        target, current, TurretConfig.MIN_ANGLE, TurretConfig.MAX_ANGLE, 80);
  }

  public static double getGoalCentricTurretTolerance(
      Translation2d goalTranslation, Pose2d robotPose, double goalCentricToleranceMeters) {
    return BaseTurretCalculator.getGoalCentricTurretTolerance(
        goalTranslation, robotPose, goalCentricToleranceMeters, TurretConfig.TURRET_TO_ROBOT);
  }
}
