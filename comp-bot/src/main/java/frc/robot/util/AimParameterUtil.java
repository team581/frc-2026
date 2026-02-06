package frc.robot.util;

import com.team581.math.ShootOnTheMove;
import com.team581.util.FeedLocation;
import com.team581.util.FieldUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.turret.TurretCalculator;

public class AimParameterUtil {
  public static double getFeedingDistance(FeedLocation feedLocation, Pose2d robot) {
    var distanceToGoal = robot.getTranslation().getDistance(feedLocation.getTranslation(robot));
    return distanceToGoal;
  }

  public static AimingParameters getFeedingParameters(
      FeedLocation feedLocation,
      Pose2d robot,
      ChassisSpeeds fieldRelativeSpeeds,
      double currentTimeofFlight) {

    var feedTranslation =
        ShootOnTheMove.getVelocityCompensatedGoal(
            feedLocation.getTranslation(robot), fieldRelativeSpeeds, currentTimeofFlight);

    var turretAngle = TurretCalculator.calculateTurretAimingAngle(robot, feedTranslation);
    var distanceToGoal = robot.getTranslation().getDistance(feedTranslation);

    return new AimingParameters(turretAngle, distanceToGoal);
  }

  public static double getScoringDistance(Pose2d robot) {
    var robotPoseInAllianceZone = FieldUtil.clampPoseToAllianceZone(robot);
    var distanceToGoal =
        robotPoseInAllianceZone
            .getTranslation()
            .getDistance(FieldUtil.HUB_POSE.getPose().getTranslation());
    return distanceToGoal;
  }

  public static AimingParameters getScoringParameters(
      Pose2d robot, ChassisSpeeds fieldRelativeSpeeds, double currentTimeofFlight) {
    var hubTranslation =
        ShootOnTheMove.getVelocityCompensatedGoal(
            FieldUtil.HUB_POSE.getPose().getTranslation(),
            fieldRelativeSpeeds,
            currentTimeofFlight);

    var robotPoseInAllianceZone = FieldUtil.clampPoseToAllianceZone(robot);
    double turretAngle =
        TurretCalculator.calculateTurretAimingAngle(robotPoseInAllianceZone, hubTranslation);
    double distanceToGoal = robotPoseInAllianceZone.getTranslation().getDistance(hubTranslation);

    return new AimingParameters(turretAngle, distanceToGoal);
  }

  public record AimingParameters(double turretAngle, double distance) {}
}
