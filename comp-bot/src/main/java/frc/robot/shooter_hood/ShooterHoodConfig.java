package frc.robot.shooter_hood;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team581.math.PolynomialRegression;
import com.team581.util.tuning.TunableInterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import java.util.Map;

public class ShooterHoodConfig {
  public static final double MAX_ANGLE = 30;
  public static final double MIN_ANGLE = 0;
  public static final double IDLE_ANGLE = 0.0;

  public static final double HOMING_VOLTAGE = 0;
  public static final double HOMING_CURRENT_THRESHOLD = 0;
  public static final double HOMING_END_POSITION = 0;

  public static final double TOLERANCE = 1;

  public static final TalonFXConfiguration MOTOR_CONFIG =
      new TalonFXConfiguration()
          // TODO: Figure out gearing ratio
          .withFeedback(
              new FeedbackConfigs().withSensorToMechanismRatio((340.0 * 42.0) / (14.0 * 8.0)))
          .withCurrentLimits(
              new CurrentLimitsConfigs().withStatorCurrentLimit(20).withSupplyCurrentLimit(20))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Brake)
                  .withInverted(InvertedValue.CounterClockwise_Positive))
          .withSlot0(new Slot0Configs().withKP(0).withKV(0).withKS(0));

  public static final InterpolatingDoubleTreeMap DISTANCE_TO_SCORE =
      TunableInterpolatingDoubleTreeMap.ofEntries(
          "ShooterHood/DistanceToScore", Map.entry(0.0, 0.0), Map.entry(10.0, 100.0));
  public static final InterpolatingDoubleTreeMap DISTANCE_TO_FEED =
      TunableInterpolatingDoubleTreeMap.ofEntries(
          "ShooterHood/DistanceToFeed", Map.entry(0.0, 0.0), Map.entry(0.0, 100.0));
  public static final PolynomialRegression SCORING_REGRESSION_MODEL =
      PolynomialRegression.quadratic("ShooterHood/ScoringRegression", DISTANCE_TO_SCORE);
  public static final PolynomialRegression FEEDING_REGRESSION_MODEL =
      PolynomialRegression.quadratic("ShooterHood/FeedingRegression", DISTANCE_TO_FEED);

  private ShooterHoodConfig() {}
}
