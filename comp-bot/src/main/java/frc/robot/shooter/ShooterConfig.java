package frc.robot.shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.team581.math.MathHelpers;
import com.team581.math.PolynomialRegression;
import com.team581.util.tuning.TunableInterpolatingDoubleTreeMap;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoubleSubscriber;
import frc.robot.config.RobotKind;
import java.util.Map;

public class ShooterConfig {
  public static final double RPM_TOLERANCE = 100.0;
  public static final double RPM_TOLERANCE_ACTIVELY_SHOOTING = 300.0;

  public static final double RPM_TOLERANCE_FEEDING = 500.0;

  public static final double GP_DETECT_CURRENT_THRESHOLD = 70.0;

  public static final double IDLE_RPM = 750;

  public static final double PIT_FUNCTIONALITY_RPM = 800;

  public static final double TEST_VOLTAGE = 6.0;

  public static final double MAX_SAFE_RPM = 6000;

  public static final Transform2d SHOOTER_TO_ROBOT = new Transform2d(0.0, 0.0, Rotation2d.k180deg);

  public static final InterpolatingDoubleTreeMap DISTANCE_TO_SCORE_RPM =
      TunableInterpolatingDoubleTreeMap.ofEntries(
          "Shooter/DistanceToScoreRPM",
          Map.entry(4.92, 1900.0),
          Map.entry(3.46, 1610.0),
          Map.entry(2.79, 1560.0),
          Map.entry(1.42, 1400.0));
  public static final InterpolatingDoubleTreeMap DISTANCE_TO_FEEDING_RPM =
      TunableInterpolatingDoubleTreeMap.ofEntries(
          "Shooter/DistanceToFeedingRPM",
          Map.entry(6.0, 1500.0),
          Map.entry(8.71, 2200.0),
          Map.entry(13.6, 3900.0));
  public static final PolynomialRegression SCORING_REGRESSION_MODEL =
      PolynomialRegression.quadratic("Shooter/ScoringRegression", DISTANCE_TO_SCORE_RPM);
  public static final PolynomialRegression FEEDING_REGRESSION_MODEL =
      PolynomialRegression.quadratic("Shooter/FeedingRegression", DISTANCE_TO_FEEDING_RPM);

  public static final InterpolatingDoubleTreeMap DISTANCE_TO_SCORE_TOF =
      TunableInterpolatingDoubleTreeMap.ofEntries(
          "Shooter/DistanceToScoreToF",
          // near
          Map.entry(1.36, 1.017),
          // tower
          Map.entry(2.42, 1.233),
          // trench
          Map.entry(3.54, 1.148),
          // corner
          Map.entry(5.5, 1.348));

  public static final InterpolatingDoubleTreeMap DISTANCE_TO_FEED_TOF =
      TunableInterpolatingDoubleTreeMap.ofEntries(
          "Shooter/DistanceToFeedToF",
          Map.entry(6.0, 1.305555556),
          Map.entry(8.71, 1.275555556),
          Map.entry(13.6, 1.530952381));

  public static final PolynomialRegression SCORING_TOF_REGRESSION_MODEL =
      PolynomialRegression.quadratic("Shooter/ScoringToFRegression", DISTANCE_TO_SCORE_TOF);
  public static final PolynomialRegression FEEDING_TOF_REGRESSION_MODEL =
      PolynomialRegression.quadratic("Shooter/FeedingToFRegression", DISTANCE_TO_FEED_TOF);
  public static final TalonFXConfiguration TOP_LEFT_MOTOR_CONFIGS =
      createMotorConfig()
          .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive))
          .withSlot0(new Slot0Configs().withKP(0.0).withKV(0.0));
  public static final TalonFXConfiguration TOP_RIGHT_MOTOR_CONFIG =
      createMotorConfig()
          .withMotorOutput(
              new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive))
          .withSlot0(
              new Slot0Configs()
                  .withKP(RobotKind.IS_COMP_BOT ? 0 : 14.0)
                  .withKS(RobotKind.IS_COMP_BOT ? 0 : 6.4)
                  .withKV(RobotKind.IS_COMP_BOT ? 0 : 0.13));
  public static final TalonFXConfiguration BOTTOM_LEFT_MOTOR_CONFIG =
      createMotorConfig()
          .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive))
          .withSlot0(new Slot0Configs().withKP(0.0).withKV(0.0));
  public static final TalonFXConfiguration BOTTOM_RIGHT_MOTOR_CONFIG =
      createMotorConfig()
          .withMotorOutput(
              new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive))
          .withSlot0(new Slot0Configs().withKP(0.0).withKV(0.0));

  public static DoubleSubscriber ACTIVE_SHOT_FF_CURRENT =
      DogLog.tunable("Shooter/ActiveShotFFCurrent", 20.0);

  public static DoubleSubscriber TURBO_MODE_FF_CURRENT =
      DogLog.tunable("Shooter/TurboShotFFCurrent", 30.0);
  // Calculated from average of CAPIN logs
  public static DoubleSubscriber FEEDER_TO_SHOOTER_TRAVEL_TIME =
      DogLog.tunable("Shooter/FeederToShooterTravelTime", 0.25);

  public static Rotation2d calculateAimingAngle(
      Translation2d shooterTranslation, Translation2d goalTranslation) {
    return MathHelpers.getDriveDirection(shooterTranslation, goalTranslation)
        .plus(SHOOTER_TO_ROBOT.getRotation());
  }

  public static double getGoalCentricTolerance(
      Translation2d goalTranslation, Pose2d shooterPose, double goalCentricToleranceMeters) {

    double distanceToGoal = shooterPose.getTranslation().getDistance(goalTranslation);
    return Math.toDegrees(Math.atan2(goalCentricToleranceMeters, distanceToGoal));
  }

  public static Pose2d getShooterPose(Pose2d robot) {
    return robot.plus(SHOOTER_TO_ROBOT);
  }

  public static ChassisSpeeds getShooterSpeeds(ChassisSpeeds robotSpeeds, double robotHeading) {
    var angularVelocity = robotSpeeds.omegaRadiansPerSecond;
    Translation2d fieldRelativeOffset =
        SHOOTER_TO_ROBOT.getTranslation().rotateBy(Rotation2d.fromDegrees(robotHeading));
    var shooterSwingX = -angularVelocity * fieldRelativeOffset.getY();
    var shooterSwingY = angularVelocity * fieldRelativeOffset.getX();
    var shooterTotalVelocityX = robotSpeeds.vxMetersPerSecond + shooterSwingX;
    var shooterTotalVelocityY = robotSpeeds.vyMetersPerSecond + shooterSwingY;
    return new ChassisSpeeds(shooterTotalVelocityX, shooterTotalVelocityY, angularVelocity);
  }

  private static TalonFXConfiguration createMotorConfig() {
    return new TalonFXConfiguration()
        .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(1.0 / 1.0))
        .withMotionMagic(
            new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(MAX_SAFE_RPM / 60.0)
                .withMotionMagicAcceleration(4000.0 / 60.0))
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(150)
                .withSupplyCurrentLimit(25))
        .withTorqueCurrent(
            new TorqueCurrentConfigs()
                .withPeakForwardTorqueCurrent(200)
                .withPeakReverseTorqueCurrent(-200));
  }

  private ShooterConfig() {}
}
