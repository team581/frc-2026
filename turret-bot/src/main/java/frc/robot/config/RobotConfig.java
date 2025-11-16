package frc.robot.config;

import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

public record RobotConfig(SwerveConfig swerve) {
  public record SwerveConfig(PhoenixPIDController snapController) {}

  public static RobotConfig get() {
    return CompConfig.COMPETITION_BOT;
  }
}
