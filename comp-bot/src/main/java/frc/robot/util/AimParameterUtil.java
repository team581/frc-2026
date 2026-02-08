package frc.robot.util;

import com.team581.math.ShootOnTheMove;
import com.team581.util.FeedLocation;
import com.team581.util.FieldUtil;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.shooter.ShooterConfig;
import frc.robot.turret.TurretCalculator;

public class AimParameterUtil {
  private static final ShootOnTheMove FEEDING_SOTM =
      new ShootOnTheMove(ShooterConfig.DISTANCE_TO_FEED_TOF);
  private static final ShootOnTheMove SCORING_SOTM =
      new ShootOnTheMove(ShooterConfig.DISTANCE_TO_SCORE_TOF);

  public static AimingParameters getFeedingParameters(
      FeedLocation feedLocation, Pose2d robot, ChassisSpeeds fieldRelativeSpeeds) {
    var feedTranslation =
        FEEDING_SOTM.getVelocityCompensatedGoal(
            robot.getTranslation(), feedLocation.getTranslation(robot), fieldRelativeSpeeds);

    var turretAngle = TurretCalculator.calculateTurretAimingAngle(robot, feedTranslation);
    var distanceToGoal = robot.getTranslation().getDistance(feedTranslation);

    return new AimingParameters(turretAngle, distanceToGoal);
  }

  public static AimingParameters getScoringParameters(
      Pose2d robot, ChassisSpeeds fieldRelativeSpeeds, double currentTimeofFlight) {
    var hubTranslationTangential =
        SCORING_SOTM.getTangentialVelocityCompensatedGoal(
            robot.getTranslation(),
            FieldUtil.HUB_POSE.getPose().getTranslation(),
            fieldRelativeSpeeds);

    var hubTranslationRadial =
        SCORING_SOTM.getRadialVelocityCompensatedGoal(
            robot.getTranslation(),
            FieldUtil.HUB_POSE.getPose().getTranslation(),
            fieldRelativeSpeeds);

    var robotPoseInAllianceZone = FieldUtil.clampPoseToAllianceZone(robot);
    double turretAngle =
        TurretCalculator.calculateTurretAimingAngle(
            robotPoseInAllianceZone, hubTranslationTangential);
    double distanceToGoal =
        robotPoseInAllianceZone.getTranslation().getDistance(hubTranslationRadial);

    DogLog.log(
        "AimParameterUtil/RadialHubTranslation",
        new Pose2d(
            hubTranslationRadial.getX(),
            hubTranslationRadial.getY(),
            hubTranslationRadial.getAngle()));
    DogLog.log(
        "AimParameterUtil/TangentialHubTranslation",
        new Pose2d(
            hubTranslationTangential.getX(),
            hubTranslationTangential.getY(),
            hubTranslationTangential.getAngle()));
    DogLog.log("AimParameterUtil/DistanceToGoal", distanceToGoal);
    DogLog.log("AimParameterUtil/TurretAngle", turretAngle);

    return new AimingParameters(turretAngle, distanceToGoal);
  }

  public record AimingParameters(double turretAngle, double distance) {}
}
