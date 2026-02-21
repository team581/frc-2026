package com.team581.swerve;

import static edu.wpi.first.units.Units.Degrees;

import com.team581.math.MathHelpers;
import com.team581.math.PolarChassisSpeeds;
import com.team581.util.FieldUtil;
import com.team581.util.FmsUtil;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.function.DoubleSupplier;

public class SwerveAssist {
  private static final double TRENCH_ASSIST_VELOCITY_THRESHOLD = 0.75;
  private static final double TRENCH_ASSIST_VELOCITY_ANGLE_TOLERANCE = 30.0;
  private static final double BUMP_ASSIST_VELOCITY_THRESHOLD = 0.5;
  private static final double BUMP_ASSIST_VELOCITY_ANGLE_TOLERANCE = 22.5;

  // Angles to round the snap to when swerve assisting
  public static final double TRENCH_SNAP_ROUND_ANGLE = 180.0;
  public static final double BUMP_SNAP_ROUND_ANGLE = 90.0;

  private static final DoubleSupplier WALL_SNAPS_VELOCITY_ANGLE_THRESHOLD =
      DogLog.tunable("Swerve/WallSnaps/VelocityAngleThresholdDegrees", 30.0, Degrees);

  private static final DoubleSupplier WALL_SNAPS_ROTATION_ANGLE_THRESHOLD =
      DogLog.tunable("Swerve/WallSnaps/RotationAngleThresholdDegrees", 40.0, Degrees);
  private static final DoubleSupplier WALL_SNAPS_DISTANCE_THRESHOLD =
      DogLog.tunable("Swerve/WallSnaps/DistanceThresholdMeters", 2.0);
  private static final DoubleSupplier MIN_ROBOT_VELOCITY_FOR_DIRECTION_SNAPS =
      DogLog.tunable("Swerve/MinRobotVelocityForDirectionSnapsMetersPerSecond", 0.5);

  private static final PIDController TRENCH_PID_CONTROLLER = new PIDController(10, 0, 0);

  public static boolean ableToBumpAssist(Pose2d robotPose, ChassisSpeeds fieldRelativeSpeeds) {
    var robotTranslation = robotPose.getTranslation();

    // Check if in bump assist zone
    if (FieldUtil.getCurrentBumpAssistZone(robotTranslation).isEmpty()) {
      return false;
    } else {
      return ableToSwerveAssist(
          robotPose,
          fieldRelativeSpeeds,
          BUMP_ASSIST_VELOCITY_THRESHOLD,
          FieldUtil.getClosestHubSideBumpPoint(robotTranslation),
          FieldUtil.getClosestTrenchSideBumpPoint(robotTranslation),
          BUMP_ASSIST_VELOCITY_ANGLE_TOLERANCE);
    }
  }

  public static boolean ableToDirectionSnap(ChassisSpeeds fieldRelativeSpeeds) {
    var robotVelocity = MathHelpers.getLinearVelocity(fieldRelativeSpeeds);
    return robotVelocity > MIN_ROBOT_VELOCITY_FOR_DIRECTION_SNAPS.getAsDouble();
  }

  public static boolean ableToTrenchAssist(Pose2d robotPose, ChassisSpeeds fieldRelativeSpeeds) {
    var robotTranslation = robotPose.getTranslation();

    // Check if in trench assist zone
    if (FieldUtil.getCurrentTrenchAssistZone(robotTranslation).isEmpty()) {
      return false;
    } else {
      return ableToSwerveAssist(
          robotPose,
          fieldRelativeSpeeds,
          TRENCH_ASSIST_VELOCITY_THRESHOLD,
          FieldUtil.getClosestAllianceZoneTrenchMidpoint(robotTranslation),
          FieldUtil.getClosestNeutralZoneTrenchMidpoint(robotTranslation),
          TRENCH_ASSIST_VELOCITY_ANGLE_TOLERANCE);
    }
  }

  public static boolean ableToWallSnap(
      Pose2d robotPose,
      ChassisSpeeds fieldRelativeSpeeds,
      Rotation2d filteredLastDriveDirection,
      double distanceToWallIntakePoint) {

    var closestWallTranslation =
        MathHelpers.getClosestPointOnRectanglePerimeter(
            robotPose.getTranslation(), FieldUtil.FIELD_BOUNDS);

    DogLog.log(
        "Swerve/WallSnaps/ClosestWallPose", new Pose2d(closestWallTranslation, Rotation2d.kZero));

    DogLog.log(
        "Swerve/WallSnaps/FilteredVelocityAngle", filteredLastDriveDirection.getDegrees(), Degrees);

    var angleToWall = MathHelpers.getDriveDirection(robotPose, closestWallTranslation);
    DogLog.log("Swerve/WallSnaps/AngleToWall", angleToWall.getDegrees(), Degrees);

    double robotAngle = robotPose.getRotation().getDegrees();

    var intakeAngleDifference = MathHelpers.angleModulus(angleToWall.getDegrees() - robotAngle);
    DogLog.log("Swerve/WallSnaps/IntakeAngleDifference", intakeAngleDifference);
    var driveAngleDifference =
        MathHelpers.angleModulus(
            angleToWall.getDegrees() - filteredLastDriveDirection.getDegrees());
    DogLog.log("Swerve/WallSnaps/DriveAngleDifference", driveAngleDifference);

    var signMisMatch =
        Math.abs(
                    MathHelpers.angleModulus(
                        filteredLastDriveDirection.getDegrees() - angleToWall.getDegrees()))
                > 1e-5
            && Math.signum(intakeAngleDifference) != Math.signum(driveAngleDifference);
    DogLog.log("Swerve/WallSnaps/SignMisMatch", signMisMatch);
    if (signMisMatch) {
      return false;
    }

    var velocityAngleTowardWall =
        MathHelpers.getLinearVelocity(fieldRelativeSpeeds) > 0.01
            && MathUtil.isNear(
                angleToWall.getDegrees(),
                filteredLastDriveDirection.getDegrees(),
                WALL_SNAPS_VELOCITY_ANGLE_THRESHOLD.getAsDouble(),
                -180,
                180);
    var rotationAngleTowardWall =
        MathUtil.isNear(
            angleToWall.getDegrees(),
            robotAngle,
            WALL_SNAPS_ROTATION_ANGLE_THRESHOLD.getAsDouble(),
            -180,
            180);

    var distanceToWallThreshold =
        distanceToWallIntakePoint < WALL_SNAPS_DISTANCE_THRESHOLD.getAsDouble();
    DogLog.log("Swerve/WallSnaps/DistanceToWall", distanceToWallThreshold);
    DogLog.log("Swerve/WallSnaps/VelocityAngleTowardWall", velocityAngleTowardWall);
    DogLog.log("Swerve/WallSnaps/RotationAngleTowardWall", rotationAngleTowardWall);
    return distanceToWallThreshold && velocityAngleTowardWall && rotationAngleTowardWall;
  }

  public static double getRoundedSnapAngle(Rotation2d robotHeading, double roundingAngle) {
    DogLog.log("SwerveAssist/TrenchAssistDebug/RobotHeading", robotHeading.getDegrees());
    DogLog.log(
        "SwerveAssist/TrenchAssistDebug/RoundedSnapAngle",
        Math.round(robotHeading.getDegrees() / roundingAngle) * roundingAngle);
    return Math.round(robotHeading.getDegrees() / roundingAngle) * roundingAngle;
  }

  public static PolarChassisSpeeds getTrenchAssistSpeeds(
      Translation2d robotTranslation, ChassisSpeeds inputSpeeds) {
    double wantedYVelocity =
        TRENCH_PID_CONTROLLER.calculate(
            robotTranslation.getY(),
            FieldUtil.getClosestAllianceZoneTrenchMidpoint(robotTranslation).getY());
    if (FmsUtil.isRedAlliance()) {
      wantedYVelocity *= -1.0;
    }

    var wantedSpeeds =
        new PolarChassisSpeeds(
            inputSpeeds.vxMetersPerSecond, wantedYVelocity, inputSpeeds.omegaRadiansPerSecond);

    var polarInputSpeeds = new PolarChassisSpeeds(inputSpeeds);

    if (polarInputSpeeds.vMetersPerSecond > 1e-5) {
      var scalar = polarInputSpeeds.vMetersPerSecond / 4.75;
      scalar = Math.min(scalar, 0.75);
      var newDirection = polarInputSpeeds.direction.interpolate(wantedSpeeds.direction, scalar);
      return new PolarChassisSpeeds(
          polarInputSpeeds.vMetersPerSecond, newDirection, wantedSpeeds.omegaRadiansPerSecond);
    }
    return polarInputSpeeds;
  }

  private static boolean ableToSwerveAssist(
      Pose2d robotPose,
      ChassisSpeeds fieldRelativeSpeeds,
      double velocityThreshold,
      Translation2d firstAssistPoint,
      Translation2d secondAssistPoint,
      double velocityAngleTolerance) {
    // Check if velocity meets threshold
    if (MathHelpers.getLinearVelocity(fieldRelativeSpeeds) <= velocityThreshold) {
      DogLog.log("SwerveAssist/VelocityThreshold", false);
      return false;
    }
    DogLog.log("SwerveAssist/VelocityThreshold", true);

    DogLog.log(
        "SwerveAssist/ClosestFirstAssistPoint",
        new Pose2d(firstAssistPoint, Rotation2d.kCCW_90deg));
    DogLog.log(
        "SwerveAssist/ClosestSecondAssistPoint",
        new Pose2d(secondAssistPoint, Rotation2d.kCCW_90deg));

    // Check if velocity angle is toward trench
    var velocityAngle = MathHelpers.getDriveDirection(fieldRelativeSpeeds);
    var angleToFirstAssistPoint = MathHelpers.getDriveDirection(robotPose, firstAssistPoint);
    var angleToSecondAssistPoint = MathHelpers.getDriveDirection(robotPose, secondAssistPoint);

    if (MathUtil.isNear(
            velocityAngle.getDegrees(),
            angleToFirstAssistPoint.getDegrees(),
            velocityAngleTolerance,
            -180.0,
            180.0)
        || MathUtil.isNear(
            velocityAngle.getDegrees(),
            angleToSecondAssistPoint.getDegrees(),
            velocityAngleTolerance,
            -180.0,
            180.0)) {
      DogLog.log("SwerveAssist/VelocityAngleTolerance", true);
      return true;
    }

    DogLog.log("SwerveAssist/VelocityAngleTolerance", false);
    return false;
  }
}
