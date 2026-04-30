package frc.robot.deploy;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.config.RobotKind;

public class DeployConfig {
  public static final double MAX_LENGTH = 11.8;
  public static final double MIN_LENGTH = 0;
  public static final double HOMING_END_POSITION_INWARD = 0;
  public static final double HOMING_END_POSITION_OUTWARD = 11.9;
  public static final double HOMING_VOLTAGE_INWARD = -2;
  public static final double HOMING_VOLTAGE_OUTWARD = 3;
  public static final double HOMING_CURRENT = RobotKind.IS_COMP_BOT ? 10.0 : 10.0;
  public static final double POSITION_TOLERANCE = 0.25;

  private static final Slot0Configs AVERAGE_GAINS =
      new Slot0Configs()
          .withKP(8.0)
          .withKI(0)
          .withKD(0.0)
          .withKG(0.0)
          .withKS(0.0)
          .withKV(0.0)
          .withKA(0);

  // Difference axis gains typically go in Slot 1
  private static final Slot1Configs DIFFERENCE_GAINS =
      new Slot1Configs().withKP(8.0).withKI(0).withKD(0.0).withKS(0.0).withKV(0.0);

  // Separate differential gains where we rail kP on differential axis to force left/right side to
  // sync up
  private static final Slot2Configs DIFFERENCE_GAINS_FIX_DESYNC =
      new Slot2Configs().withKP(100.0).withKI(0).withKD(0.0).withKS(0.0).withKV(0.0);

  public static final TalonFXConfiguration LEFT_MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withFeedback(
              new FeedbackConfigs()
                  .withSensorToMechanismRatio((40.0 / 8.0) * (1 / (Math.PI * (2 * 0.5)))))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(
                      RobotKind.IS_COMP_BOT ? NeutralModeValue.Brake : NeutralModeValue.Coast)
                  .withInverted(InvertedValue.Clockwise_Positive))
          .withCurrentLimits(
              new CurrentLimitsConfigs().withStatorCurrentLimit(50).withSupplyCurrentLimit(18))
          .withTorqueCurrent(
              new TorqueCurrentConfigs()
                  .withPeakForwardTorqueCurrent(100)
                  .withPeakReverseTorqueCurrent(-35))
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicCruiseVelocity(200.0)
                  .withMotionMagicAcceleration(50.0))
          .withSlot0(AVERAGE_GAINS)
          .withSlot1(DIFFERENCE_GAINS)
          .withSlot2(DIFFERENCE_GAINS_FIX_DESYNC);

  public static final TalonFXConfiguration RIGHT_MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withFeedback(
              new FeedbackConfigs()
                  .withSensorToMechanismRatio((40.0 / 8.0) * (1 / (Math.PI * (2 * 0.5)))))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(
                      RobotKind.IS_COMP_BOT ? NeutralModeValue.Brake : NeutralModeValue.Coast)
                  .withInverted(InvertedValue.CounterClockwise_Positive))
          .withCurrentLimits(
              new CurrentLimitsConfigs().withStatorCurrentLimit(50).withSupplyCurrentLimit(18))
          .withTorqueCurrent(
              new TorqueCurrentConfigs()
                  .withPeakForwardTorqueCurrent(100)
                  .withPeakReverseTorqueCurrent(-35))
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicCruiseVelocity(200.0)
                  .withMotionMagicAcceleration(50.0))
          .withSlot0(AVERAGE_GAINS)
          .withSlot1(DIFFERENCE_GAINS)
          .withSlot2(DIFFERENCE_GAINS_FIX_DESYNC);

  private DeployConfig() {}
}
