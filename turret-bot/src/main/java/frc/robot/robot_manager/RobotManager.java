package frc.robot.robot_manager;

import com.team581.util.state_machines.StateMachineSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.turret.TurretSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.vision.VisionSubsystem;

public class RobotManager extends StateMachineSubsystem<RobotState> {
  public final LocalizationSubsystem localization;
  public final SwerveSubsystem swerve;
  public final TurretSubsystem turret;
  public final VisionSubsystem vision;

  public RobotManager(
      LocalizationSubsystem localization,
      SwerveSubsystem swerve,
      TurretSubsystem turret,
      VisionSubsystem vision) {
    super(SubsystemPriority.ROBOT_MANAGER, RobotState.IDLE);
    this.turret = turret;
    this.localization = localization;
    this.swerve = swerve;
    this.vision = vision;
  }

  @Override
  protected RobotState getNextState(RobotState currentState) {
    return switch (currentState) {
      default -> currentState;
    };
  }

  @Override
  protected void afterTransition(RobotState newState) {
    switch (newState) {
      case AUTO_AIM -> swerve.normalDriveRequest();
      case MANUAL_AIM -> swerve.normalDriveRequest();
      case IDLE -> swerve.normalDriveRequest();
      case LOCKED -> swerve.normalDriveRequest();
    }
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();

    vision.addTurretObservation(
        Timer.getFPGATimestamp(), Rotation2d.fromDegrees(turret.getAngle()));

    MechanismVisualizer.log(localization.getPose(), turret.getAngle());
  }
}
