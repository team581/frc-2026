package frc.robot.localization;

import com.ctre.phoenix6.Utils;
import com.team581.math.MathHelpers;
import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.generated.RobotTunerConstants.TunerSwerveDrivetrain;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;

public class LocalizationSubsystem extends StateMachineSubsystem<LocalizationState> {
  private final SwerveSubsystem swerve;
  private final TunerSwerveDrivetrain drivetrain;
  private Pose2d robotPose = Pose2d.kZero;

  public LocalizationSubsystem(SwerveSubsystem swerve, TunerSwerveDrivetrain drivetrain) {
    super(SubsystemPriority.LOCALIZATION, LocalizationState.DEFAULT_STATE);
    this.swerve = swerve;
    this.drivetrain = drivetrain;
  }

  @Override
  protected void collectInputs() {
    robotPose = drivetrain.getState().Pose;
  }

  public Pose2d getPose() {
    return robotPose;
  }

  public Pose2d getPose(double timestamp) {
    var newTimestamp = Utils.fpgaToCurrentTime(timestamp);
    return drivetrain.samplePoseAt(newTimestamp).orElseGet(this::getPose);
  }

  public Pose2d getLookaheadPose(double lookahead) {
    return MathHelpers.poseLookahead(getPose(), swerve.getFieldRelativeSpeeds(), lookahead);
  }

  @Override
  public void whileInState(LocalizationState currentState) {
    DogLog.log("Localization/EstimatedPose", getPose());
  }

  public void zeroGyro() {
    drivetrain.seedFieldCentric();
  }

  public void resetPose(Pose2d estimatedPose) {
    drivetrain.resetPose(estimatedPose);
  }
}
