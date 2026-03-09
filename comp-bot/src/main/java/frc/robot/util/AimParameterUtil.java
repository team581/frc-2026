package frc.robot.util;

import com.team581.math.MathHelpers;
import com.team581.math.ShootOnTheMove;
import com.team581.util.FeedLocation;
import com.team581.util.FieldUtil;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import frc.robot.shooter.ShooterConfig;
import frc.robot.turret.TurretCalculator;

public class AimParameterUtil {

  private static final ShootOnTheMove SCORING_SOTM =
      new ShootOnTheMove(ShooterConfig.DISTANCE_TO_SCORE_TOF);

  private static final ShootOnTheMove FEEDING_SOTM =
      new ShootOnTheMove(ShooterConfig.DISTANCE_TO_FEED_TOF);

  private static final double SCORING_TURRET_TOLERANCE = Units.inchesToMeters(20);

  private static final double FALLBACK_FEEDING_TURRET_TOLERANCE = 1;
  private static final double FEEDING_FALLBACK_DISTANCE_TO_GOAL = 8.0;

  private static final DoubleSubscriber UPCOMING_TURRET_ANGLE_LOOKAHEAD =
      DogLog.tunable("AimingParameters/UpcomingTurretAngleLookahead", 0.5);

  public static AimingParameters getFallbackFeedingParameters(
      FeedLocation feedLocation, Pose2d robotPose, ChassisSpeeds fieldRelativeSpeeds) {

    var turretAngle =
        TurretCalculator.calculateTurretAimingAngle(
            robotPose, robotPose.plus(new Transform2d(-1, 0, Rotation2d.kZero)).getTranslation());

    var upcomingTurretAngle =
        TurretCalculator.calculateTurretAimingAngle(
            MathHelpers.getLookaheadPose(
                robotPose, fieldRelativeSpeeds, UPCOMING_TURRET_ANGLE_LOOKAHEAD.get()),
            robotPose.plus(new Transform2d(-1, 0, Rotation2d.kZero)).getTranslation());

    return new AimingParameters(
        turretAngle,
        FEEDING_FALLBACK_DISTANCE_TO_GOAL,
        FALLBACK_FEEDING_TURRET_TOLERANCE,
        -fieldRelativeSpeeds.omegaRadiansPerSecond,
        upcomingTurretAngle);
  }

  public static AimingParameters getFeedingParameters(
      FeedLocation feedLocation, Pose2d robotPose, ChassisSpeeds fieldRelativeSpeeds) {
    // Calculate translation of turret on the field
    var turretPose = TurretCalculator.getTurretPose(robotPose);
    var turretTranslation = turretPose.getTranslation();

    // Calculate speeds of turret (x, y, omega)
    var turretFieldRelativeSpeeds =
        TurretCalculator.getTurretChassisSpeeds(
            fieldRelativeSpeeds, robotPose.getRotation().getDegrees());

    Translation2d feedTranslation = feedLocation.getTranslation(robotPose);

    // Get velocity compensated goals
    var separatedVelocityCompensatedGoal =
        FEEDING_SOTM.getSeparatedVelocityCompensatedGoalWithEffectiveTof(
            turretTranslation, feedTranslation, turretFieldRelativeSpeeds);

    // Calculate fully compensated distance to goal for shooter, hood, and turret
    var compensatedGoal = separatedVelocityCompensatedGoal.fullyCompensatedGoal();
    var fullyCompensatedDistanceToGoal = turretTranslation.getDistance(compensatedGoal);

    var turretAngle = TurretCalculator.calculateTurretAimingAngle(robotPose, compensatedGoal);

    // Use same goal for goal centric turret tolerance
    var turretTolerance =
        TurretCalculator.getGoalCentricTurretTolerance(
            compensatedGoal, robotPose, SCORING_TURRET_TOLERANCE);

    // Calculate translational FF for turret to account for linear turret velocity
    var realDistanceToGoal = turretTranslation.getDistance(feedTranslation);
    double tangentialVelocity = separatedVelocityCompensatedGoal.tangentialVelocity();
    double translationalFF = tangentialVelocity / realDistanceToGoal;

    // Sum translationall FF and rotational FF
    double totalTurretFFRadians =
        -translationalFF - turretFieldRelativeSpeeds.omegaRadiansPerSecond;

    var upcomingTurretAngle =
        TurretCalculator.calculateTurretAimingAngle(
            MathHelpers.getLookaheadPose(
                turretPose, fieldRelativeSpeeds, UPCOMING_TURRET_ANGLE_LOOKAHEAD.get()),
            compensatedGoal);

    return new AimingParameters(
        turretAngle,
        fullyCompensatedDistanceToGoal,
        turretTolerance,
        totalTurretFFRadians,
        upcomingTurretAngle);
  }

  public static AimingParameters getScoringParameters(
      Pose2d robotPose, ChassisSpeeds fieldRelativeSpeeds) {
    // Calculate translation of turret on the field
    var turretPose = TurretCalculator.getTurretPose(robotPose);

    var turretTranslation = turretPose.getTranslation();

    // Calculate speeds of turret (x, y, omega)
    var turretFieldRelativeSpeeds =
        TurretCalculator.getTurretChassisSpeeds(
            fieldRelativeSpeeds, robotPose.getRotation().getDegrees());

    Translation2d hubTranslation = FieldUtil.HUB_POSE.getTranslation();

    // Get velocity compensated goals
    var separatedVelocityCompensatedGoal =
        SCORING_SOTM.getSeparatedVelocityCompensatedGoalWithEffectiveTof(
            turretTranslation, hubTranslation, turretFieldRelativeSpeeds);

    // Calculate fully compensated distance to goal for shooter, hood, and turret
    var compensatedGoal = separatedVelocityCompensatedGoal.fullyCompensatedGoal();
    var fullyCompensatedDistanceToGoal = turretTranslation.getDistance(compensatedGoal);

    var turretAngle = TurretCalculator.calculateTurretAimingAngle(robotPose, compensatedGoal);

    // Use same goal for goal centric turret tolerance
    var turretTolerance =
        TurretCalculator.getGoalCentricTurretTolerance(
            compensatedGoal, robotPose, SCORING_TURRET_TOLERANCE);

    // Calculate translational FF for turret to account for linear turret velocity
    var realDistanceToGoal = turretTranslation.getDistance(hubTranslation);
    double tangentialVelocity = separatedVelocityCompensatedGoal.tangentialVelocity();
    double translationalFF = tangentialVelocity / realDistanceToGoal;

    // Sum translationall FF and rotational FF
    double totalTurretFFRadians =
        -translationalFF - turretFieldRelativeSpeeds.omegaRadiansPerSecond;

    var upcomingTurretAngle =
        TurretCalculator.calculateTurretAimingAngle(
            MathHelpers.getLookaheadPose(
                turretPose, fieldRelativeSpeeds, UPCOMING_TURRET_ANGLE_LOOKAHEAD.get()),
            compensatedGoal);

    return new AimingParameters(
        turretAngle,
        fullyCompensatedDistanceToGoal,
        turretTolerance,
        totalTurretFFRadians,
        upcomingTurretAngle);
  }

  public static AimingParameters getTurretStuckScoringParameters(
      Pose2d robot, double turretAngle, ChassisSpeeds fieldRelativeSpeeds) {
    var turretFieldRelativeSpeeds =
        TurretCalculator.getTurretChassisSpeeds(
            fieldRelativeSpeeds, robot.getRotation().getDegrees());
    var hubTranslation =
        SCORING_SOTM.getVelocityCompensatedGoal(
            robot.getTranslation(),
            FieldUtil.HUB_POSE.getPose().getTranslation(),
            turretFieldRelativeSpeeds);

    var turretCompenstatedRobotPose = TurretCalculator.getTurretPose(robot);
    double distanceToGoal =
        turretCompenstatedRobotPose.getTranslation().getDistance(hubTranslation);
    var angle =
        MathHelpers.getDriveDirection(turretCompenstatedRobotPose, hubTranslation)
            .minus(Rotation2d.fromDegrees(turretAngle));

    var turretTolerance =
        TurretCalculator.getGoalCentricTurretTolerance(
            hubTranslation, robot, SCORING_TURRET_TOLERANCE);
    return new AimingParameters(
        angle.getDegrees(),
        distanceToGoal,
        turretTolerance,
        -turretFieldRelativeSpeeds.omegaRadiansPerSecond,
        turretAngle);
  }

  public record AimingParameters(
      double turretAngle,
      double distance,
      double turretTolerance,
      double turretFeedForwardRadians,
      double upcomingTurretAngle) {}
}
