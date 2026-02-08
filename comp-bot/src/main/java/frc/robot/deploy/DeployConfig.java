package frc.robot.deploy;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class DeployConfig {
  // TODO: These are just placeholders - we should update these when we find out these lengths
  public static final double MAX_LENGTH = 12;
  public static final double MIN_LENGTH = 0;
  public static final double HOMING_END_POSITION = 0;
  public static final double HOMING_VOLTAGE = 0;
  public static final double HOMING_CURRENT = 40.0;
  public static final double CAPACITY_DISTANCE_THRESHOLD = 0.0;
  public static final double POSITION_TOLERANCE = 0.25;

  public static final TalonFXConfiguration LEFT_MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withFeedback(
              new FeedbackConfigs()
                  .withSensorToMechanismRatio((40.0 / 8.0) * (Math.PI * (2 * 0.5))))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Brake)
                  .withInverted(InvertedValue.Clockwise_Positive))
          .withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(30))
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicCruiseVelocity(0.0)
                  .withMotionMagicAcceleration(0.0))
          .withSlot0(new Slot0Configs().withKP(0).withKV(0).withKG(0));
  public static final TalonFXConfiguration RIGHT_MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withFeedback(
              new FeedbackConfigs()
                  .withSensorToMechanismRatio((40.0 / 8.0) * (Math.PI * (2 * 0.5))))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Brake)
                  .withInverted(InvertedValue.CounterClockwise_Positive))
          .withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(30))
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicCruiseVelocity(0.0)
                  .withMotionMagicAcceleration(0.0))
          .withSlot0(new Slot0Configs().withKP(0).withKV(0).withKG(0));
  // TODO: Discuss/set CANrange config during bringup
  public static final CANrangeConfiguration CAN_RANGE_CONFIG = new CANrangeConfiguration();
}
