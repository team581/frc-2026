package frc.robot.shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team581.math.PolynomialRegression;
import com.team581.util.tuning.TunableInterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.config.RobotKind;
import java.util.Map;

public class ShooterConfig {
  public static final double SELF_TEST_LEFT_MOTOR_EXPECTED_RPM = 2500;
  public static final double SELF_TEST_LEFT_MOTOR_RPM_TOLERANCE = 250;
  public static final double SELF_TEST_LEFT_MOTOR_EXPECTED_CURRENT = 10.0;
  public static final double SELF_TEST_LEFT_MOTOR_CURRENT_TOLERANCE = 5;

  public static final double SELF_TEST_RIGHT_MOTOR_EXPECTED_RPM = 2500;
  public static final double SELF_TEST_RIGHT_MOTOR_RPM_TOLERANCE = 250;
  public static final double SELF_TEST_RIGHT_MOTOR_EXPECTED_CURRENT = 10.0;
  public static final double SELF_TEST_RIGHT_MOTOR_CURRENT_TOLERANCE = 5;
  public static final int RPM_TOLERANCE = 150;
  public static final int RPM_TOLERANCE_FEEDING = 1000;

  public static final double IDLE_RPM = 400;

  public static final double TEST_VOLTAGE = 6.0;

  public static final double MAX_SAFE_RPM = 6000;

  public static final InterpolatingDoubleTreeMap DISTANCE_TO_SCORE_RPM =
      RobotKind.IS_COMP_BOT
          ? TunableInterpolatingDoubleTreeMap.ofEntries(
              "Shooter/DistanceToScoreRPM",
              Map.entry(5.551, 2350.0),
              Map.entry(4.25, 1850.0),
              Map.entry(1.89, 1400.0),
              Map.entry(1.25, 1400.0))
          : TunableInterpolatingDoubleTreeMap.ofEntries(
              "Shooter/DistanceToScoreRPM",
              Map.entry(5.551, 2350.0),
              Map.entry(3.42, 2000.0),
              Map.entry(2.33, 1850.0),
              Map.entry(1.41, 1650.0));
  public static final InterpolatingDoubleTreeMap DISTANCE_TO_FEEDING_RPM =
      TunableInterpolatingDoubleTreeMap.ofEntries(
          "Shooter/DistanceToFeedingRPM",
          Map.entry(5.551, 2000.0),
          Map.entry(3.42, 1500.0),
          Map.entry(2.33, 1000.0),
          Map.entry(1.41, 700.0));
  public static final PolynomialRegression SCORING_REGRESSION_MODEL =
      PolynomialRegression.quadratic("Shooter/ScoringRegression", DISTANCE_TO_SCORE_RPM);
  public static final PolynomialRegression FEEDING_REGRESSION_MODEL =
      PolynomialRegression.quadratic("Shooter/FeedingRegression", DISTANCE_TO_FEEDING_RPM);

  public static final InterpolatingDoubleTreeMap DISTANCE_TO_SCORE_TOF =
      TunableInterpolatingDoubleTreeMap.ofEntries(
          "Shooter/DistanceToScoreToF",
          Map.entry(1.85, 1.035897436),
          Map.entry(3.47, 1.095833333),
          Map.entry(5.5, 1.163333333));

  public static final InterpolatingDoubleTreeMap DISTANCE_TO_FEED_TOF =
      TunableInterpolatingDoubleTreeMap.ofEntries(
          "Shooter/DistanceToFeedToF",
          Map.entry(3.57, 1.488888889),
          Map.entry(1.25, 1.491666667),
          Map.entry(5.5, 1.396666667));
  public static final TalonFXConfiguration LEFT_MOTOR_CONFIGS =
      new TalonFXConfiguration()
          .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(1.0 / 1.0))
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicCruiseVelocity(MAX_SAFE_RPM / 60.0)
                  .withMotionMagicAcceleration(4000.0 / 60.0))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withSupplyCurrentLimitEnable(true)
                  .withStatorCurrentLimitEnable(true)
                  .withStatorCurrentLimit(100)
                  .withSupplyCurrentLimit(100))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Coast)
                  .withInverted(InvertedValue.CounterClockwise_Positive))
          .withSlot0(
              new Slot0Configs().withKP(RobotKind.IS_COMP_BOT ? 0.55 : 0.7).withKV(0.127)
              // .withKD(0.00015)
              )
          .withVoltage(new VoltageConfigs().withPeakReverseVoltage(0))
          .withTorqueCurrent(
              new TorqueCurrentConfigs()
                  .withPeakForwardTorqueCurrent(200)
                  .withPeakReverseTorqueCurrent(0));
  public static final TalonFXConfiguration RIGHT_MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(1.0 / 1.0))
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicCruiseVelocity(MAX_SAFE_RPM / 60.0)
                  .withMotionMagicAcceleration(4000.0 / 60.0))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withSupplyCurrentLimitEnable(true)
                  .withStatorCurrentLimitEnable(true)
                  .withStatorCurrentLimit(100)
                  .withSupplyCurrentLimit(100))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Coast)
                  .withInverted(InvertedValue.Clockwise_Positive))
          .withSlot0(
              new Slot0Configs().withKP(RobotKind.IS_COMP_BOT ? 0.55 : 0.7).withKV(0.127)
              // .withKD(0.00015)
              )
          .withVoltage(new VoltageConfigs().withPeakReverseVoltage(0))
          .withTorqueCurrent(
              new TorqueCurrentConfigs()
                  .withPeakForwardTorqueCurrent(200)
                  .withPeakReverseTorqueCurrent(0));

  private ShooterConfig() {}
}
