package frc.robot.climber;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team581.autos.Point;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class ClimberConfig {
  public static final double MIN_HEIGHT = 0.0;
  public static final double MAX_HEIGHT = 30.0;

  public static final TalonFXConfiguration MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(1))
          .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
          .withCurrentLimits(
              new CurrentLimitsConfigs().withStatorCurrentLimit(30).withStatorCurrentLimit(30))
          .withSlot0(new Slot0Configs().withKP(0.0).withKV(0.0).withKG(0.0));

  // TODO: UPDATE WITH REAL GOAL LOCATIONS FROM CAD
  public static final Point CLIMB_LEFT_LOCATION =
      Point.ofRed(new Pose2d(14.968, 3.867, Rotation2d.k180deg));
  public static final Point CLIMB_RIGHT_LOCATION =
      Point.ofRed(new Pose2d(14.968, 4.748, Rotation2d.k180deg));

  public static final double TOLERANCE = 0.1;
}
