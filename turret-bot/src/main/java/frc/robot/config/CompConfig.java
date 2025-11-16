package frc.robot.config;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import frc.robot.config.RobotConfig.SwerveConfig;
import frc.robot.generated.RobotTunerConstants;

class CompConfig {
  public static final CANBus CANIVORE_CAN = RobotTunerConstants.kCANBus;
  public static final CANBus RIO_CAN = new CANBus("rio");

  public static final RobotConfig COMPETITION_BOT =
      new RobotConfig(new SwerveConfig(new PhoenixPIDController(5.75, 0, 0)));

  private CompConfig() {}
}
