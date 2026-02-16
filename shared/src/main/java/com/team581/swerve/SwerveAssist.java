package com.team581.swerve;

import static edu.wpi.first.units.Units.Degrees;

import com.team581.math.MathHelpers;
import com.team581.math.PolarChassisSpeeds;
import com.team581.util.FieldUtil;
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
    if (FieldUtil.getCurrentBumpAssistZone(robotPose.getTranslation()).isEmpty()) {
      return false;
    }

    // Check if velocity meets threshold
    if (MathHelpers.getLinearVelocity(fieldRelativeSpeeds) <= BUMP_ASSIST_VELOCITY_THRESHOLD) {
      DogLog.log("SwerveAssist/Bump/VelocityThreshold", false);
      return false;
    }
    DogLog.log("SwerveAssist/Bump/VelocityThreshold", true);

    // Check if velocity angle is toward bump
    var hubSideBumpAssistPoint = FieldUtil.getClosestHubSideBumpPoint(robotTranslation);
    var trenchSideBumpAssistPoint = FieldUtil.getClosestTrenchSideBumpPoint(robotTranslation);
    DogLog.log(
        "SwerveAssist/Bump/ClosestHubSideBumpPoint",
        new Pose2d(hubSideBumpAssistPoint, Rotation2d.kZero));
    DogLog.log(
        "SwerveAssist/Bump/ClosestTremchSideBumpPoint",
        new Pose2d(trenchSideBumpAssistPoint, Rotation2d.kZero));

    var velocityAngle = MathHelpers.getDriveDirection(fieldRelativeSpeeds);
    var angleToHubSideBumpPoint = MathHelpers.getDriveDirection(robotPose, hubSideBumpAssistPoint);
    var angleToTrenchSideBumpPoint =
        MathHelpers.getDriveDirection(robotPose, trenchSideBumpAssistPoint);

    if (MathUtil.isNear(
            velocityAngle.getDegrees(),
            angleToHubSideBumpPoint.getDegrees(),
            BUMP_ASSIST_VELOCITY_ANGLE_TOLERANCE,
            -180.0,
            180.0)
        || MathUtil.isNear(
            velocityAngle.getDegrees(),
            angleToTrenchSideBumpPoint.getDegrees(),
            BUMP_ASSIST_VELOCITY_ANGLE_TOLERANCE,
            -180.0,
            180.0)) {
      DogLog.log("SwerveAssist/Bump/VelocityAngleTolerance", true);
      return true;
    }
    DogLog.log("SwerveAssist/Bump/VelocityAngleTolerance", false);
    return false;
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
    }

    // Check if velocity meets threshold
    if (MathHelpers.getLinearVelocity(fieldRelativeSpeeds) <= TRENCH_ASSIST_VELOCITY_THRESHOLD) {
      DogLog.log("SwerveAssist/Trench/VelocityThreshold", false);
      return false;
    }
    DogLog.log("SwerveAssist/Trench/VelocityThreshold", true);

    var allianceZoneAssistPoint = FieldUtil.getClosestAllianceZoneTrenchMidpoint(robotTranslation);
    var neutralZoneAssistPoint = FieldUtil.getClosestNeutralZoneTrenchMidpoint(robotTranslation);
    DogLog.log(
        "SwerveAssist/Trench/ClosestAllianceZoneTrenchMidpoint",
        new Pose2d(allianceZoneAssistPoint, Rotation2d.kCCW_90deg));
    DogLog.log(
        "SwerveAssist/Trench/ClosestNeutralZoneTrenchMidpoint",
        new Pose2d(neutralZoneAssistPoint, Rotation2d.kCCW_90deg));

    // Check if velocity angle is toward trench
    var velocityAngle = MathHelpers.getDriveDirection(fieldRelativeSpeeds);
    var angleToAllianceZoneAssistPoint =
        MathHelpers.getDriveDirection(robotPose, allianceZoneAssistPoint);
    var angleToNeutralZoneAssistPoint =
        MathHelpers.getDriveDirection(robotPose, neutralZoneAssistPoint);

    if (MathUtil.isNear(
            velocityAngle.getDegrees(),
            angleToAllianceZoneAssistPoint.getDegrees(),
            TRENCH_ASSIST_VELOCITY_ANGLE_TOLERANCE,
            -180.0,
            180.0)
        || MathUtil.isNear(
            velocityAngle.getDegrees(),
            angleToNeutralZoneAssistPoint.getDegrees(),
            TRENCH_ASSIST_VELOCITY_ANGLE_TOLERANCE,
            -180.0,
            180.0)) {
      DogLog.log("SwerveAssist/Trench/VelocityAngleTolerance", true);
      return true;
    }
    DogLog.log("SwerveAssist/Trench/VelocityAngleTolerance", false);
    return false;
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
    return Math.round(robotHeading.getDegrees() / roundingAngle) * roundingAngle;
  }

  public static PolarChassisSpeeds getTrenchAssistSpeeds(
      Translation2d robotTranslation, ChassisSpeeds inputSpeeds) {
    double wantedYVelocity =
        -TRENCH_PID_CONTROLLER.calculate(
            robotTranslation.getY(),
            FieldUtil.getClosestAllianceZoneTrenchMidpoint(robotTranslation).getY());

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
}
