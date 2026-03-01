package frc.robot.dye_rotor;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team581.util.tuning.TunableInterpolatingDoubleTreeMap;
import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.DoubleSubscriber;
import java.util.Map;

public class DyeRotorConfig {
  public static final Debouncer IS_SHOOTING_DEBOUNCER = new Debouncer(1.0);

  // TODO: Measure this number
  public static final double RPM_TOLERANCE_SHOOTING = 10;

  public static final double HOMING_END_POSITION = 180;

  public static final DoubleSubscriber JAM_CURRENT_THRESHOLD =
      DogLog.tunable("DyeRotor/Horizontal/JamCurrentThreshold", 55.0);

  public static final InterpolatingDoubleTreeMap DISTANCE_TO_SCORE_BPS =
      TunableInterpolatingDoubleTreeMap.ofEntries(
          "DyeRotor/DistanceToScoreBPS",
          Map.entry(5.56, 10.0),
          Map.entry(3.56, 75.0),
          Map.entry(1.69, 100.0));
  public static final InterpolatingDoubleTreeMap DISTANCE_TO_FEED_BPS =
      TunableInterpolatingDoubleTreeMap.ofEntries(
          "DyeRotor/DistanceToFeedBPS",
          Map.entry(9.56, 100.0),
          Map.entry(3.56, 100.0),
          Map.entry(1.69, 100.0));

  public static final TalonFXConfiguration ROTOR_MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withFeedback(
              new FeedbackConfigs()
                  .withSensorToMechanismRatio((40.0 * 36.0 * 84.0) / (8.0 * 18.0 * 18.0)))
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicCruiseVelocity(10.0)
                  .withMotionMagicAcceleration(10.0))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withSupplyCurrentLimitEnable(true)
                  .withStatorCurrentLimitEnable(true)
                  .withStatorCurrentLimit(60.0)
                  .withSupplyCurrentLimit(60.0))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Coast)
                  .withInverted(InvertedValue.Clockwise_Positive))
          .withSlot0(new Slot0Configs().withKP(10.0).withKV(4.65).withKS(0.0));

  public static final TalonFXConfiguration VERTICAL_MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(12.0 / 18.0))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withSupplyCurrentLimitEnable(true)
                  .withStatorCurrentLimitEnable(true)
                  .withStatorCurrentLimit(100)
                  .withSupplyCurrentLimit(100))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Coast)
                  .withInverted(InvertedValue.Clockwise_Positive));

  public static final TalonFXConfiguration HORIZONTAL_MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(15.0 / 15.0))
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicCruiseVelocity(0.0)
                  .withMotionMagicAcceleration(0.0))
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
          .withSlot0(new Slot0Configs().withKP(0.0).withKV(0.0).withKS(0.0));
  public static boolean ROTOR_STOP;

  private DyeRotorConfig() {}
}
