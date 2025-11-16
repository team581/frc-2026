package frc.robot.localization;

import com.ctre.phoenix6.Utils;
import com.team581.math.MathHelpers;
import com.team581.trailblazer.LocalizationBase;
import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;

public class LocalizationSubsystem extends StateMachineSubsystem<LocalizationState>
    implements LocalizationBase {
  private final SwerveSubsystem swerve;
  private Pose2d robotPose = Pose2d.kZero;

  public LocalizationSubsystem(SwerveSubsystem swerve) {
    super(SubsystemPriority.LOCALIZATION, LocalizationState.DEFAULT_STATE);
    this.swerve = swerve;
  }

  @Override
  protected void collectInputs() {
    robotPose = swerve.drivetrain.getState().Pose;
  }

  @Override
  public Pose2d getPose() {
    return robotPose;
  }

  public Pose2d getPose(double timestamp) {
    var newTimestamp = Utils.fpgaToCurrentTime(timestamp);
    return swerve.drivetrain.samplePoseAt(newTimestamp).orElseGet(this::getPose);
  }

  public Pose2d getLookaheadPose(double lookahead) {
    return MathHelpers.poseLookahead(getPose(), swerve.getFieldRelativeSpeeds(), lookahead);
  }

  @Override
  public void whileInState(LocalizationState currentState) {
    DogLog.log("Localization/EstimatedPose", getPose());
  }

  public void zeroGyro() {
    swerve.drivetrain.seedFieldCentric();
  }

  public void resetPose(Pose2d estimatedPose) {
    swerve.drivetrain.resetPose(estimatedPose);
  }
}
