package frc.robot.deploy;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class DeployConfig {

  public static final double MAX_LENGTH = 12.75;
  public static final double MIN_LENGTH = 0;
  public static final double HOMING_END_POSITION_INWARD = 0;
  public static final double HOMING_END_POSITION_OUTWARD = 12.75;
  public static final double HOMING_VOLTAGE_INWARD = -2;
  public static final double HOMING_VOLTAGE_OUTWARD = 2;
  public static final double HOMING_CURRENT = 30.0;
  public static final double CAPACITY_DISTANCE_THRESHOLD = 0.0;
  public static final double POSITION_TOLERANCE = 0.25;
  public static final double HOPPER_SHUFFLE_DISTANCE = 3.0;
  public static final double NOT_UPDATING_TIMEOUT = 3.0;

  private static final Slot0Configs AVERAGE_GAINS =
      new Slot0Configs()
          .withKP(3)
          .withKI(0)
          .withKD(0.0)
          .withKG(0.0)
          .withKS(0.0)
          .withKV(0.0)
          .withKA(0);
  // Difference axis gains typically go in Slot 1
  private static final Slot1Configs DIFFERENCE_GAINS =
      new Slot1Configs().withKP(3).withKI(0).withKD(0.0).withKS(0.0).withKV(0.0);

  public static final TalonFXConfiguration LEFT_MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withFeedback(
              new FeedbackConfigs()
                  .withSensorToMechanismRatio((40.0 / 8.0) * (1 / (Math.PI * (2 * 0.5)))))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Coast)
                  .withInverted(InvertedValue.Clockwise_Positive))
          .withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(30))
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicCruiseVelocity(200.0)
                  .withMotionMagicAcceleration(300.0))
          .withSlot0(AVERAGE_GAINS)
          .withSlot1(DIFFERENCE_GAINS);
  public static final TalonFXConfiguration RIGHT_MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withFeedback(
              new FeedbackConfigs()
                  .withSensorToMechanismRatio((40.0 / 8.0) * (1 / (Math.PI * (2 * 0.5)))))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Coast)
                  .withInverted(InvertedValue.CounterClockwise_Positive))
          .withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(30))
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicCruiseVelocity(200.0)
                  .withMotionMagicAcceleration(300.0))
          .withSlot0(AVERAGE_GAINS)
          .withSlot1(DIFFERENCE_GAINS);
  // TODO: Discuss/set CANrange config during bringup
  public static final CANrangeConfiguration CAN_RANGE_CONFIG = new CANrangeConfiguration();

  public static final double HIGH_CAPACITY_THRESHOLD = 10;
  public static final double MEDIUM_CAPACITY_THRESHOLD = 5;
}
