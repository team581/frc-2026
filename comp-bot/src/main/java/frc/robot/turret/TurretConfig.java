package frc.robot.turret;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import frc.robot.config.RobotKind;

public class TurretConfig {
  public static final double MIN_ANGLE = RobotKind.IS_COMP_BOT ? -370.0 : -355.0;
  public static final double MAX_ANGLE = RobotKind.IS_COMP_BOT ? 70.0 : 5.0;
  public static final double OUT_OF_BOUNDS_THRESHOLD = 1.0;
  public static final double HOMING_END_POSITION = MIN_ANGLE;

  public static final double MOTOR_TO_TURRET = ((220.0 / 14.0) * (36.0 / 10.0));

  // CAL NUMBER
  public static final double ENCODER_CAL_OFFSET =
      RobotKind.IS_COMP_BOT ? -0.18408203125 : 0.6796875;
  public static final double MOTOR_ROTOR_CAL_OFFSET = RobotKind.IS_COMP_BOT ? 0.157227 : 0.0;

  public static final double MOTOR_ROTATION_RESOLUTION = 1 / MOTOR_TO_TURRET;
  public static final double ENCODER_TO_TURRET =
      (float) 220.0 / 25.0 * 8.0 / 30.0 * 8.0 / 35.0; // Encoder rot to turret rot

  // Turret 2d transform relative to robot center
  public static final Transform2d TURRET_TO_ROBOT =
      new Transform2d(Units.inchesToMeters(0.5), Units.inchesToMeters(0.0), Rotation2d.kZero);

  public static final TalonFXConfiguration MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(MOTOR_TO_TURRET))
          // TODO: Switch back to brake mode once bringup concluded
          .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast))
          .withCurrentLimits(
              new CurrentLimitsConfigs().withStatorCurrentLimit(40).withSupplyCurrentLimit(20))
          .withVoltage(new VoltageConfigs().withPeakForwardVoltage(10).withPeakReverseVoltage(-10))
          .withSlot0(
              new Slot0Configs().withKP(200).withKV(6.0).withKG(0.0).withKD(1.7).withKS(0.2));
  public static final CANcoderConfiguration ENCODER_CONFIG =
      new CANcoderConfiguration()
          .withMagnetSensor(
              new MagnetSensorConfigs()
                  .withMagnetOffset(ENCODER_CAL_OFFSET)
                  .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
                  .withAbsoluteSensorDiscontinuityPoint(0.2));

  private TurretConfig() {}
}
