package frc.robot.intake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeConfig {
  public static final TalonFXConfiguration MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(InvertedValue.CounterClockwise_Positive)
                  .withNeutralMode(NeutralModeValue.Coast))
          .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(36.0 / 12.0))
          .withSlot0(new Slot0Configs().withKP(10.0).withKV(4.65).withKS(0.0))
          .withCurrentLimits(
              new CurrentLimitsConfigs().withStatorCurrentLimit(80).withSupplyCurrentLimit(80));
}
