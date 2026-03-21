package frc.robot.shooter_hood;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team581.math.PolynomialRegression;
import com.team581.util.tuning.TunableInterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import java.util.Map;

public class ShooterHoodConfig {
  /**
   * This is the effective angle of the shooter hood relative to the floor when the shooter hood is
   * fully retracted.
   */
  // TODO: Update angle from horizontal numbers
  public static final double ANGLE_FROM_HORIZONTAL = 21.4;

  public static final double MAX_ANGLE = 43.0;
  public static final double MIN_ANGLE = ANGLE_FROM_HORIZONTAL + 1;
  public static final double IDLE_ANGLE = ANGLE_FROM_HORIZONTAL + 2;

  // TODO: Update Homing numbers
  public static final double HOMING_VOLTAGE = -1;
  public static final double HOMING_CURRENT_THRESHOLD = 10;
  public static final double HOMING_END_POSITION = ANGLE_FROM_HORIZONTAL;

  // TODO: Update tolerance numbers
  public static final double TOLERANCE = 1;
  public static final double FEEDING_TOLERANCE = 5;

  // TODO: Update gear ratios
  public static final TalonFXConfiguration MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withFeedback(
              new FeedbackConfigs().withSensorToMechanismRatio((336.0 * 42.0) / (14.0 * 8.0)))
          .withCurrentLimits(
              new CurrentLimitsConfigs().withStatorCurrentLimit(20).withSupplyCurrentLimit(10))
          .withVoltage(new VoltageConfigs().withPeakForwardVoltage(10).withPeakReverseVoltage(-10))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Coast)
                  .withInverted(InvertedValue.CounterClockwise_Positive))
          .withSlot0(new Slot0Configs().withKP(350).withKV(0).withKS(0));

  // TODO: Update interpolating map numbers later
  public static final InterpolatingDoubleTreeMap DISTANCE_TO_SCORE =
      TunableInterpolatingDoubleTreeMap.ofEntries(
          "ShooterHood/DistanceToScore",
          Map.entry(5.5 + 0.25 + 0.15, 32.0),
          Map.entry(3.54 + 0.25 + 0.15, 27.0),
          Map.entry(2.42 + 0.25 + 0.15, 23.5),
          Map.entry(1.36 + 0.25 + 0.15, 21.5));
  public static final InterpolatingDoubleTreeMap DISTANCE_TO_FEED =
      TunableInterpolatingDoubleTreeMap.ofEntries(
          "ShooterHood/DistanceToFeed",
          Map.entry(6.0, 30.0),
          Map.entry(8.71, 35.0),
          Map.entry(13.6, 43.0));
  public static final PolynomialRegression SCORING_REGRESSION_MODEL =
      PolynomialRegression.quadratic("ShooterHood/ScoringRegression", DISTANCE_TO_SCORE);
  public static final PolynomialRegression FEEDING_REGRESSION_MODEL =
      PolynomialRegression.quadratic("ShooterHood/FeedingRegression", DISTANCE_TO_FEED);

  private ShooterHoodConfig() {}
}
