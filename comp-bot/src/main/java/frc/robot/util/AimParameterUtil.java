package frc.robot.util;

import com.team581.math.MathHelpers;
import com.team581.math.ShootOnTheMove;
import com.team581.util.FeedLocation;
import com.team581.util.FieldUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.shooter.ShooterConfig;
import frc.robot.turret.TurretCalculator;
import frc.robot.turret.TurretConfig;

public class AimParameterUtil {
  private static final ShootOnTheMove FEEDING_SOTM =
      new ShootOnTheMove(ShooterConfig.DISTANCE_TO_FEED_TOF);
  private static final ShootOnTheMove SCORING_SOTM =
      new ShootOnTheMove(ShooterConfig.DISTANCE_TO_SCORE_TOF);

  private static final double SCORING_TURRET_TOLERANCE = Units.inchesToMeters(10);
  private static final double FEEDING_TURRET_TOLERANCE = Units.inchesToMeters(20);

  public static AimingParameters getFeedingParameters(
      FeedLocation feedLocation, Pose2d robot, ChassisSpeeds fieldRelativeSpeeds) {
    var feedTranslation =
        FEEDING_SOTM.getVelocityCompensatedGoal(
            robot.getTranslation(), feedLocation.getTranslation(robot), fieldRelativeSpeeds);
    var turretAngle = TurretCalculator.calculateTurretAimingAngle(robot, feedTranslation);
    var distanceToGoal = robot.getTranslation().getDistance(feedTranslation);

    var turretTolerance =
        TurretCalculator.getGoalCentricTurretTolerance(
            feedTranslation, robot, FEEDING_TURRET_TOLERANCE);

    return new AimingParameters(turretAngle, distanceToGoal, turretTolerance);
  }

  public static AimingParameters getScoringParameters(
      Pose2d robot, ChassisSpeeds fieldRelativeSpeeds) {
    var hubTranslation =
        SCORING_SOTM.getVelocityCompensatedGoal(
            robot.getTranslation(),
            FieldUtil.HUB_POSE.getPose().getTranslation(),
            fieldRelativeSpeeds);

    var robotPoseInAllianceZone = FieldUtil.clampPoseToAllianceZone(robot);
    double turretAngle =
        TurretCalculator.calculateTurretAimingAngle(robotPoseInAllianceZone, hubTranslation);
    double distanceToGoal = robotPoseInAllianceZone.getTranslation().getDistance(hubTranslation);
    var turretTolerance =
        TurretCalculator.getGoalCentricTurretTolerance(
            hubTranslation, robot, SCORING_TURRET_TOLERANCE);

    return new AimingParameters(turretAngle, distanceToGoal, turretTolerance);
  }

  public static AimingParameters getTurretStuckScoringParameters(
      Pose2d robot, double turretAngle, ChassisSpeeds fieldRelativeSpeeds) {
    var hubTranslation =
        SCORING_SOTM.getVelocityCompensatedGoal(
            robot.getTranslation(),
            FieldUtil.HUB_POSE.getPose().getTranslation(),
            fieldRelativeSpeeds);

    var turretCompenstatedRobotPose = robot.plus(TurretConfig.TURRET_TO_ROBOT);
    var robotPoseInAllianceZone = FieldUtil.clampPoseToAllianceZone(turretCompenstatedRobotPose);
    double distanceToGoal = robotPoseInAllianceZone.getTranslation().getDistance(hubTranslation);
    var angle =
        MathHelpers.getDriveDirection(robotPoseInAllianceZone, hubTranslation)
            .minus(Rotation2d.fromDegrees(turretAngle));

    var turretTolerance =
        TurretCalculator.getGoalCentricTurretTolerance(
            hubTranslation, robot, SCORING_TURRET_TOLERANCE);
    return new AimingParameters(angle.getDegrees(), distanceToGoal, turretTolerance);
  }

  public record AimingParameters(double turretAngle, double distance, double turretTolerance) {}
}
