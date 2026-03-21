package frc.robot.util;

import com.team581.math.MathHelpers;
import com.team581.math.ShootOnTheMove;
import com.team581.util.FeedLocation;
import com.team581.util.FieldUtil;
import com.team581.util.FmsUtil;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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

  private static final double FEEDING_FALLBACK_DISTANCE_TO_GOAL = 8.0;

  private static final DoubleSubscriber UPCOMING_TURRET_ANGLE_LOOKAHEAD =
      DogLog.tunable("AimingParameters/UpcomingTurretAngleLookahead", 0.5);

  public static AimingParameters getFallbackFeedingParameters(Rotation2d robotRotation) {
    var fieldRelativeGoal = FmsUtil.isRedAlliance() ? Rotation2d.kZero : Rotation2d.k180deg;
    var turretAngle = fieldRelativeGoal.minus(robotRotation);

    return new AimingParameters(
        turretAngle.getDegrees(), FEEDING_FALLBACK_DISTANCE_TO_GOAL, 5, 0, 0);
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

    // Sum translational FF and rotational FF
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

    // Sum translational FF and rotational FF
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

  public static AimingParameters getTurretStuckFeedingParameters(
      FeedLocation feedLocation,
      Pose2d robot,
      double turretAngle,
      ChassisSpeeds fieldRelativeSpeeds) {

    var turretPose = TurretCalculator.getTurretPose(robot);
    var turretFieldRelativeSpeeds =
        TurretCalculator.getTurretChassisSpeeds(
            fieldRelativeSpeeds, robot.getRotation().getDegrees());

    var feedTranslation =
        FEEDING_SOTM
            .getSeparatedVelocityCompensatedGoal(
                turretPose.getTranslation(),
                feedLocation.getTranslation(turretPose),
                turretFieldRelativeSpeeds)
            .fullyCompensatedGoal();

    double tangentialVelocity =
        SCORING_SOTM
            .getSeparatedVelocityCompensatedGoal(
                turretPose.getTranslation(),
                FieldUtil.HUB_POSE.getTranslation(),
                turretFieldRelativeSpeeds)
            .tangentialVelocity();

    var realDistanceToGoal =
        turretPose.getTranslation().getDistance(feedLocation.getTranslation(robot));

    double translationalFF = tangentialVelocity / realDistanceToGoal;

    // Sum translational FF and rotational FF
    double totalTurretFFRadians = -translationalFF;

    double distanceToGoal = turretPose.getTranslation().getDistance(feedTranslation);
    var angle =
        MathHelpers.getDriveDirection(turretPose, feedTranslation)
            .minus(Rotation2d.fromDegrees(turretAngle));

    return new AimingParameters(
        angle.getDegrees(), distanceToGoal, 1.0, totalTurretFFRadians, turretAngle);
  }

  public static AimingParameters getTurretStuckScoringParameters(
      Pose2d robot, double turretAngle, ChassisSpeeds fieldRelativeSpeeds) {

    var turretPose = TurretCalculator.getTurretPose(robot);
    var turretFieldRelativeSpeeds =
        TurretCalculator.getTurretChassisSpeeds(
            fieldRelativeSpeeds, robot.getRotation().getDegrees());

    var scoreTranslation =
        SCORING_SOTM
            .getSeparatedVelocityCompensatedGoal(
                turretPose.getTranslation(),
                FieldUtil.HUB_POSE.getTranslation(),
                turretFieldRelativeSpeeds)
            .fullyCompensatedGoal();

    // Calculate translational FF for turret to account for linear turret velocity
    var realDistanceToGoal =
        turretPose.getTranslation().getDistance(FieldUtil.HUB_POSE.getTranslation());
    double tangentialVelocity =
        SCORING_SOTM
            .getSeparatedVelocityCompensatedGoal(
                robot.getTranslation(), FieldUtil.HUB_POSE.getTranslation(), fieldRelativeSpeeds)
            .tangentialVelocity();
    double translationalFF = tangentialVelocity / realDistanceToGoal;

    // Sum translational FF and rotational FF
    double totalTurretFFRadians = -translationalFF;

    double distanceToGoal = turretPose.getTranslation().getDistance(scoreTranslation);
    var angle =
        MathHelpers.getDriveDirection(turretPose, scoreTranslation)
            .minus(Rotation2d.fromDegrees(turretAngle));

    return new AimingParameters(
        angle.getDegrees(), distanceToGoal, 1.0, totalTurretFFRadians, turretAngle);
  }

  public record AimingParameters(
      double turretAngle,
      double distance,
      double turretTolerance,
      double turretFeedForwardRadians,
      double upcomingTurretAngle) {}
}
