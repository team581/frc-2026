package frc.robot.util;

import com.team581.math.ShootOnTheMove;
import com.team581.util.FeedLocation;
import com.team581.util.FieldUtil;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.shooter.ShooterConfig;
import frc.robot.turret.TurretCalculator;

public class AimParameterUtil {
  private static final ShootOnTheMove FEEDING_SOTM =
      new ShootOnTheMove(ShooterConfig.DISTANCE_TO_FEED_TOF);
  private static final ShootOnTheMove SCORING_SOTM =
      new ShootOnTheMove(ShooterConfig.DISTANCE_TO_SCORE_TOF);

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
        new Pose2d(separatedVelocityCompensatedGoal.tangentiallyCompensatedGoal(), Rotation2d.kZero));

    var turretAngle =
        TurretCalculator.calculateTurretAimingAngle(
            robotPose, separatedVelocityCompensatedGoal.tangentiallyCompensatedGoal());
    var distanceToGoal =
        robotPose
            .getTranslation()
            .getDistance(separatedVelocityCompensatedGoal.radiallyCompensatedGoal());

    return new AimingParameters(turretAngle, distanceToGoal);
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
        new Pose2d(separatedVelocityCompensatedGoal.tangentiallyCompensatedGoal(), Rotation2d.kZero));

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

    return new AimingParameters(turretAngle, distanceToGoal);
  }

  public record AimingParameters(double turretAngle, double distance) {}
}
