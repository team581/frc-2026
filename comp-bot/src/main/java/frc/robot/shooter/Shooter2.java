package frc.robot.shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import com.team581.util.state_machines.StateMachineSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;

public class Shooter2 extends StateMachineSubsystem<ShooterState> {

  public Shooter2(TalonFX leftMotor, TalonFX rightMotor) {
    super(SubsystemPriority.SHOOTER, ShooterState.IDLE);

    leftMotor.getConfigurator().apply(ShooterConfig.LEFT_MOTOR_CONFIGS);
    rightMotor.getConfigurator().apply(ShooterConfig.RIGHT_MOTOR_CONFIG);
  }

  public void scoreRequest(double distance) {

    setStateFromRequest(ShooterState.SCORE);
  }
}
