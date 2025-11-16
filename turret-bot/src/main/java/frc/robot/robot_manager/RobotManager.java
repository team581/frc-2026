package frc.robot.robot_manager;

import com.team581.util.state_machines.StateMachineSubsystem;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;

public class RobotManager extends StateMachineSubsystem<RobotState> {
  public final LocalizationSubsystem localization;
  public final SwerveSubsystem swerve;

  public RobotManager(LocalizationSubsystem localization, SwerveSubsystem swerve) {
    super(SubsystemPriority.ROBOT_MANAGER, RobotState.PLACEHOLDER_STATE);
    this.localization = localization;
    this.swerve = swerve;
  }

  @Override
  protected void afterTransition(RobotState newState) {
    switch (newState) {
      case PLACEHOLDER_STATE -> swerve.normalDriveRequest();
    }
  }
}
