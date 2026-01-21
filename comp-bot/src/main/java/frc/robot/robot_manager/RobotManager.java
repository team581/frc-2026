package frc.robot.robot_manager;

import com.team581.util.FieldUtil;
import com.team581.util.state_machines.StateMachineSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.localization.Localization;
import frc.robot.shooter_hood.ShooterHood;
import frc.robot.swerve.Swerve;
import frc.robot.util.scheduling.SubsystemPriority;

public class RobotManager extends StateMachineSubsystem<RobotState> {
  public final Localization localization;
  public final Swerve swerve;
  private final ShooterHood shooterHood;

  private Pose2d robotPose = Pose2d.kZero;
  private boolean nearTrench = false;

  public RobotManager(ShooterHood shooterHood, Localization localization, Swerve swerve) {
    super(SubsystemPriority.ROBOT_MANAGER, RobotState.IDLE);
    this.shooterHood = shooterHood;
    this.localization = localization;
    this.swerve = swerve;
  }

  @Override
  protected void afterTransition(RobotState newState) {
    switch (newState) {
      case IDLE -> {
        swerve.normalDriveRequest();
        shooterHood.idleRequest();
      }
    }
  }

  @Override
  protected void whileInState(RobotState state) {
    switch (state) {
      case IDLE -> {
        if (nearTrench) {
          shooterHood.idleRequest();
        } else {
          shooterHood.scoreRequest(0);
        }
      }
      default -> {}
    }
  }

  @Override
  protected void collectInputs() {
    robotPose = localization.getPose();
    nearTrench =
        FieldUtil.TRENCH_ASSIST_ZONES.stream()
            .anyMatch(trench -> trench.contains(robotPose.getTranslation()));
  }
}
