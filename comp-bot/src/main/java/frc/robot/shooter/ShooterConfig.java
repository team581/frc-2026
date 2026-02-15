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
import edu.wpi.first.math.util.Units;
import java.util.Map;

public class ShooterConfig {
  public static final int RPM_TOLERANCE_SHOOTER = 100;

  public static final double MAX_SAFE_RPM = 5000;

  public static final InterpolatingDoubleTreeMap DISTANCE_TO_SCORE_RPM =
      TunableInterpolatingDoubleTreeMap.ofEntries(
          "Shooter/DistanceToScoreRPM", Map.entry(Units.inchesToMeters(5.5), 4200.0));
  public static final InterpolatingDoubleTreeMap DISTANCE_TO_FEEDING_RPM =
      TunableInterpolatingDoubleTreeMap.ofEntries(
          "Shooter/DistanceToFeedingRPM",
          Map.entry(1.0, 2000.0),
          Map.entry(2.0, 3500.0),
          Map.entry(5.0, 5000.0));
  public static final PolynomialRegression SCORING_REGRESSION_MODEL =
      PolynomialRegression.quadratic("Shooter/ScoringRegression", DISTANCE_TO_SCORE_RPM);
  public static final PolynomialRegression FEEDING_REGRESSION_MODEL =
      PolynomialRegression.quadratic("Shooter/FeedingRegression", DISTANCE_TO_FEEDING_RPM);

  public static final InterpolatingDoubleTreeMap DISTANCE_TO_SCORE_TOF =
      TunableInterpolatingDoubleTreeMap.ofEntries(
          "Shooter/DistanceToScoreToF",
          Map.entry(Units.inchesToMeters(36.0), 0.2),
          Map.entry(Units.inchesToMeters(96.0), 0.5));

  public static final InterpolatingDoubleTreeMap DISTANCE_TO_FEED_TOF =
      TunableInterpolatingDoubleTreeMap.ofEntries(
          "Shooter/DistanceToFeedToF", Map.entry(Units.inchesToMeters(57.0), 0.0));

  public static final TalonFXConfiguration LEFT_MOTOR_CONFIGS =
      new TalonFXConfiguration()
          .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(1))
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
          .withSlot0(new Slot0Configs().withKP(0.8).withKV(0.1225).withKS(0.0))
          .withVoltage(new VoltageConfigs().withPeakReverseVoltage(0))
          .withTorqueCurrent(
              new TorqueCurrentConfigs()
                  .withPeakForwardTorqueCurrent(200)
                  .withPeakReverseTorqueCurrent(0));
  public static final TalonFXConfiguration RIGHT_MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(1))
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
          .withSlot0(new Slot0Configs().withKP(0.8).withKV(0.1225).withKS(0.0))
          .withVoltage(new VoltageConfigs().withPeakReverseVoltage(0))
          .withTorqueCurrent(
              new TorqueCurrentConfigs()
                  .withPeakForwardTorqueCurrent(200)
                  .withPeakReverseTorqueCurrent(0));

  private ShooterConfig() {}
}
