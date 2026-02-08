package frc.robot.dye_rotor;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.networktables.DoubleSubscriber;

public class DyeRotorConfig {
  public static final Debouncer debouncer = new Debouncer(1.0);
  public static final int RPM_TOLERANCE_HORIZONTAL = 100;

  // TODO:Get this number
  public static final double RPM_TOLERANCE_SHOOTING = 10;
  public static final DoubleSubscriber JAM_CURRENT_THRESHOLD =
      DogLog.tunable("DyeRotor/Horizontal/JamCurrentThreshold", 75.0);

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
                  .withStatorCurrentLimit(100)
                  .withSupplyCurrentLimit(100))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Coast)
                  .withInverted(InvertedValue.Clockwise_Positive))
          .withSlot0(new Slot0Configs().withKP(0.0).withKV(0.0).withKS(0.0).withKA(0.0));

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
                  .withInverted(InvertedValue.CounterClockwise_Positive));

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

  private DyeRotorConfig() {}
}
