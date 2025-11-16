package frc.robot.imu;


import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.team581.mechanisms.imu.BaseImuSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;

public class ImuSubsystem extends BaseImuSubsystem {
  public ImuSubsystem(SwerveDrivetrain<?, ?, ?> drivetrain) {
    super(SubsystemPriority.IMU, drivetrain);
  }
}
