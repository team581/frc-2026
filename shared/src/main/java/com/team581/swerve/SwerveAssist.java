package com.team581.swerve;

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
import edu.wpi.first.math.util.Units;
import java.util.function.DoubleSupplier;

public class SwerveAssist {
  // General swerve assist values
  private static final double TRENCH_ASSIST_VELOCITY_THRESHOLD = 0.75;
  private static final Rotation2d TRENCH_ASSIST_VELOCITY_ANGLE_TOLERANCE =
      Rotation2d.fromDegrees(30.0);
  private static final double BUMP_ASSIST_VELOCITY_THRESHOLD = 0.5;
  private static final Rotation2d BUMP_ASSIST_VELOCITY_ANGLE_TOLERANCE =
      Rotation2d.fromDegrees(22.5);
  private static final Rotation2d WALL_SNAP_VELOCITY_ANGLE_TOLERANCE = Rotation2d.fromDegrees(40.0);

  // Wall snap values
  private static final Rotation2d WALL_SNAP_OFFSET = Rotation2d.fromDegrees(30.0);
  private static final double WALL_SNAP_PROXIMITY_THRESHOLD = Units.inchesToMeters(45.0);

  // Angles to round the snap to when swerve assisting
  public static final Rotation2d TRENCH_SNAP_ROUND_ANGLE = Rotation2d.fromDegrees(180.0);
  public static final Rotation2d BUMP_SNAP_ROUND_ANGLE = Rotation2d.fromDegrees(90.0);
  private static final Rotation2d WALL_SNAP_ROUND_ANGLE = Rotation2d.fromDegrees(180.0);

  private static final DoubleSupplier MIN_ROBOT_VELOCITY_FOR_DIRECTION_SNAPS =
      DogLog.tunable("Swerve/MinRobotVelocityForDirectionSnapsMetersPerSecond", 0.5);

  private static final PIDController SWERVE_ASSIST_PID_CONTROLLER = new PIDController(10, 0, 0);

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
      Pose2d robotPose, ChassisSpeeds fieldRelativeSpeeds, Rotation2d snapAngle) {
    // Check if we are in a corner
    if (FieldUtil.getCurrentWallSnapCornerZone(robotPose.getTranslation()).isPresent()) {
      DogLog.log("SwerveAssist/WallSnaps/Checks/AbleToCornerTransition", true);
      return true;
    }
    DogLog.log("SwerveAssist/WallSnaps/Checks/AbleToCornerTransition", false);

    // Check if we are close to a wall
    var closestWallTranslation =
        MathHelpers.getClosestPointOnRectanglePerimeter(
            robotPose.getTranslation(), FieldUtil.FIELD_BOUNDS);
    // If the closest wall is a driver station wall, the y component will be equal to the robot's
    var closestWallIsADriverStationWall =
        Math.abs(robotPose.getY() - closestWallTranslation.getY()) < 1e-5;
    if ((closestWallIsADriverStationWall
            && Math.abs(robotPose.getTranslation().getX() - closestWallTranslation.getX())
                > WALL_SNAP_PROXIMITY_THRESHOLD)
        || (!closestWallIsADriverStationWall
            && Math.abs(robotPose.getTranslation().getY() - closestWallTranslation.getY())
                > WALL_SNAP_PROXIMITY_THRESHOLD)) {
      DogLog.log("SwerveAssist/WallSnaps/Checks/CloseToWallCheck", false);
      return false;
    }
    DogLog.log("SwerveAssist/WallSnaps/Checks/CloseToWallCheck", true);

    // Check if we are driving fast enough in the direction of the intake parallel to the wall
    var roundedDriveDirection =
        getRoundedSnapAngle(
            MathHelpers.getDriveDirection(fieldRelativeSpeeds), WALL_SNAP_ROUND_ANGLE);
    if (closestWallIsADriverStationWall) {
      // Still want to round drive direction to 180.0 degrees, but rotated 90.0 degrees to be
      // parallel w/ DS wall
      roundedDriveDirection =
          getRoundedSnapAngle(
                  MathHelpers.getDriveDirection(fieldRelativeSpeeds)
                      .minus(Rotation2d.fromDegrees(90.0)),
                  WALL_SNAP_ROUND_ANGLE)
              .plus(Rotation2d.fromDegrees(90.0));
    }
    if (!MathUtil.isNear(
        robotPose.getRotation().getDegrees(),
        roundedDriveDirection.getDegrees(),
        WALL_SNAP_VELOCITY_ANGLE_TOLERANCE.getDegrees(),
        -180.0,
        180.0)) {
      DogLog.log("SwerveAssist/WallSnaps/Checks/IntakeDriveDirectionCheck", false);
      return false;
    } else {
      DogLog.log("SwerveAssist/WallSnaps/Checks/IntakeDriveDirectionCheck", true);
      return true;
    }
  }

  public static Rotation2d getRoundedSnapAngle(Rotation2d robotHeading, Rotation2d roundingAngle) {
    // No rounding method exists for Rotation2ds; instead convert to a double in degrees, use
    // Math.round(), then convert back to a Rotation2d
    return Rotation2d.fromDegrees(
        Math.round(robotHeading.getDegrees() / roundingAngle.getDegrees())
            * roundingAngle.getDegrees());
  }

  public static PolarChassisSpeeds getTrenchAssistSpeeds(
      Translation2d robotTranslation, ChassisSpeeds inputSpeeds) {
    double wantedYVelocity =
        SWERVE_ASSIST_PID_CONTROLLER.calculate(
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

  public static Rotation2d getWallSnapAngle(
      Translation2d robotTranslation, ChassisSpeeds fieldRelativeSpeeds, boolean inCorner) {
    // get direction toward wall, then apply offset of snap round angle in that direction
    var closestWallTranslation =
        MathHelpers.getClosestPointOnRectanglePerimeter(robotTranslation, FieldUtil.FIELD_BOUNDS);
    // If the closest wall is a driver station wall, the y component will be equal to the robot's
    var closestWallIsADriverStationWall =
        Math.abs(robotTranslation.getY() - closestWallTranslation.getY()) < 1e-5;
    var angleToWall = closestWallTranslation.minus(robotTranslation).getAngle();
    var driveDirection = MathHelpers.getDriveDirection(fieldRelativeSpeeds);
    var roundedSnapAngle = Rotation2d.kZero;
    // For DS wall, still round snap to 180.0 degrees, but rotated 90.0 degrees to be parallel w/ DS
    // wall
    if (inCorner) {
      roundedSnapAngle =
          closestWallIsADriverStationWall
              ? getRoundedSnapAngle(
                  angleToWall.minus(Rotation2d.fromDegrees(180.0)), WALL_SNAP_ROUND_ANGLE)
              : getRoundedSnapAngle(
                      angleToWall
                          .minus(Rotation2d.fromDegrees(180.0))
                          .minus(Rotation2d.fromDegrees(90.0)),
                      WALL_SNAP_ROUND_ANGLE)
                  .plus(Rotation2d.fromDegrees(90.0));
    } else {
      roundedSnapAngle =
          closestWallIsADriverStationWall
              ? getRoundedSnapAngle(
                      driveDirection.minus(Rotation2d.fromDegrees(90.0)), WALL_SNAP_ROUND_ANGLE)
                  .plus(Rotation2d.fromDegrees(90.0))
              : getRoundedSnapAngle(driveDirection, WALL_SNAP_ROUND_ANGLE);
    }

    // Check which direction we are going relative to the wall
    var direction = 0;
    if (inCorner) {
      // Signed difference between drive direction and snap angle
      double delta = driveDirection.minus(roundedSnapAngle).getRadians();

      // Normalize to [-pi, pi]
      delta = Math.atan2(Math.sin(delta), Math.cos(delta));

      direction = (delta > 0) ? 1 : -1;
    } else {
      if (closestWallIsADriverStationWall) {
        if (angleToWall.plus(roundedSnapAngle).getDegrees() > 0) {
          direction = -1;
        } else {
          direction = 1;
        }
      } else {
        if (angleToWall.plus(roundedSnapAngle).getDegrees() > 0) {
          direction = 1;
        } else {
          direction = -1;
        }
      }
    }

    return roundedSnapAngle.plus(WALL_SNAP_OFFSET.times(direction));
  }

  private static boolean ableToSwerveAssist(
      Pose2d robotPose,
      ChassisSpeeds fieldRelativeSpeeds,
      double velocityThreshold,
      Translation2d firstAssistPoint,
      Translation2d secondAssistPoint,
      Rotation2d velocityAngleTolerance) {
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
            velocityAngleTolerance.getDegrees(),
            -180.0,
            180.0)
        || MathUtil.isNear(
            velocityAngle.getDegrees(),
            angleToSecondAssistPoint.getDegrees(),
            velocityAngleTolerance.getDegrees(),
            -180.0,
            180.0)) {
      DogLog.log("SwerveAssist/VelocityAngleTolerance", true);
      return true;
    }

    DogLog.log("SwerveAssist/VelocityAngleTolerance", false);
    return false;
  }
}
