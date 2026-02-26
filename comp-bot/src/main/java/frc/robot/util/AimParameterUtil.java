package frc.robot.util;

import com.team581.math.MathHelpers;
import com.team581.math.ShootOnTheMove;
import com.team581.util.FeedLocation;
import com.team581.util.FieldUtil;
import dev.doglog.DogLog;
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
      FeedLocation feedLocation, Pose2d robotPose, ChassisSpeeds fieldRelativeSpeeds) {
    var robotTranslation = robotPose.getTranslation();
    var separatedVelocityCompensatedGoal =
        FEEDING_SOTM.getSeparatedVelocityCompensatedGoal(
            robotTranslation, feedLocation.getTranslation(robotPose), fieldRelativeSpeeds);

    DogLog.log(
        "ShootOnTheMove/Feeding/RadialCompensatedGoal",
        new Pose2d(separatedVelocityCompensatedGoal.radiallyCompensatedGoal(), Rotation2d.kZero));
    DogLog.log(
        "ShootOnTheMove/Feeding/TangentialCompensatedGoal",
        new Pose2d(
            separatedVelocityCompensatedGoal.tangentiallyCompensatedGoal(), Rotation2d.kZero));

    var turretAngle =
        TurretCalculator.calculateTurretAimingAngle(
            robotPose, separatedVelocityCompensatedGoal.tangentiallyCompensatedGoal());
    var distanceToGoal =
        robotPose
            .getTranslation()
            .getDistance(separatedVelocityCompensatedGoal.radiallyCompensatedGoal());

    var turretTolerance =
        TurretCalculator.getGoalCentricTurretTolerance(
            separatedVelocityCompensatedGoal.tangentiallyCompensatedGoal(),
            robotPose,
            FEEDING_TURRET_TOLERANCE);

    return new AimingParameters(turretAngle, distanceToGoal, turretTolerance);
  }

  public static AimingParameters getScoringParameters(
      Pose2d robotPose, ChassisSpeeds fieldRelativeSpeeds) {
    var robotTranslation = robotPose.getTranslation();
    var separatedVelocityCompensatedGoal =
        SCORING_SOTM.getSeparatedVelocityCompensatedGoal(
            robotTranslation, FieldUtil.HUB_POSE.getTranslation(), fieldRelativeSpeeds);

    DogLog.log(
        "ShootOnTheMove/Scoring/RadialCompensatedGoal",
        new Pose2d(separatedVelocityCompensatedGoal.radiallyCompensatedGoal(), Rotation2d.kZero));
    DogLog.log(
        "ShootOnTheMove/Scoring/TangentialCompensatedGoal",
        new Pose2d(
            separatedVelocityCompensatedGoal.tangentiallyCompensatedGoal(), Rotation2d.kZero));

    var robotPoseInAllianceZone = FieldUtil.clampPoseToAllianceZone(robotPose);
    var turretAngle =
        TurretCalculator.calculateTurretAimingAngle(
            robotPoseInAllianceZone,
            separatedVelocityCompensatedGoal.tangentiallyCompensatedGoal());
    var distanceToGoal =
        robotPoseInAllianceZone
            .getTranslation()
            .getDistance(separatedVelocityCompensatedGoal.radiallyCompensatedGoal());

    DogLog.log("AimParameterUtil/DistanceToGoal", distanceToGoal);
    DogLog.log("AimParameterUtil/TurretAngle", turretAngle);

    var turretTolerance =
        TurretCalculator.getGoalCentricTurretTolerance(
            separatedVelocityCompensatedGoal.tangentiallyCompensatedGoal(),
            robotPose,
            SCORING_TURRET_TOLERANCE);

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
