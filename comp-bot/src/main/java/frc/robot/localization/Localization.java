package frc.robot.localization;

import com.ctre.phoenix6.Utils;
import com.team581.localization.TrustFactor;
import com.team581.util.state_machines.StateMachineSubsystem;
import com.team581.vision.results.TagResult;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generated.CompTunerConstants.TunerSwerveDrivetrain;
import frc.robot.imu.Imu;
import frc.robot.swerve.Swerve;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.vision.Vision;

public class Localization extends StateMachineSubsystem<LocalizationState> {
  private static final double LATENCY_CONSTANT = 0.0;
  private final Swerve swerve;
  private final TunerSwerveDrivetrain drivetrain;
  private final Vision vision;
  private final Imu imu;
  private final TrustFactor trustFactor = new TrustFactor();

  private final Field2d field2d = new Field2d();

  private Pose2d robotPose = Pose2d.kZero;

  public Localization(Swerve swerve, TunerSwerveDrivetrain drivetrain, Vision vision, Imu imu) {
    super(SubsystemPriority.LOCALIZATION, LocalizationState.DEFAULT_STATE);
    this.swerve = swerve;
    this.vision = vision;
    this.drivetrain = drivetrain;
    this.imu = imu;

    SmartDashboard.putData("Field", field2d);
  }

  public Pose2d getLookaheadPose(double lookahead) {
    var current = getPose();
    var velocity = swerve.getFieldRelativeSpeeds();
    var x = current.getX() + velocity.vxMetersPerSecond * lookahead;
    var y = current.getY() + velocity.vyMetersPerSecond * lookahead;
    var theta =
        current
            .getRotation()
            .plus(Rotation2d.fromRadians(velocity.omegaRadiansPerSecond * lookahead));

    return new Pose2d(x, y, theta);
  }

  public Pose2d getPose() {
    return robotPose;
  }

  public Pose2d getPose(double timestamp) {
    var newTimestamp = Utils.fpgaToCurrentTime(timestamp);
    return drivetrain.samplePoseAt(newTimestamp).orElseGet(this::getPose);
  }

  public double getTrustFactor() {
    return trustFactor.get();
  }

  public boolean isLost() {
    return trustFactor.isLost();
  }

  public boolean isTrustworthy() {
    return trustFactor.isTrustworthy();
  }

  public void resetPose(Pose2d estimatedPose) {
    drivetrain.resetPose(estimatedPose);
    trustFactor.seededPose();
  }

  @Override
  public void whileInState(LocalizationState currentState) {
    DogLog.log("Localization/EstimatedPose", getPose());
    DogLog.log("Localization/TrustFactor", getTrustFactor());
    field2d.setRobotPose(robotPose);
  }

  public void zeroGyro() {
    drivetrain.seedFieldCentric();
    trustFactor.reset();
  }

  private void ingestTagResult(TagResult result) {
    DogLog.timestamp("Localization/IngestTagResult");
    trustFactor.tagSeen(result.standardDevs().getData()[0]);
    var visionPose = result.pose();

    if (!vision.seenTagRecentlyForReset()) {
      resetPose(visionPose);
    }
    swerve.drivetrain.addVisionMeasurement(
        visionPose,
        Utils.fpgaToCurrentTime(result.timestamp() - (LATENCY_CONSTANT / 1000)),
        result.standardDevs());
  }

  @Override
  protected void collectInputs() {
    vision.getAdjustedTurretLimelighTagResult().ifPresent(this::ingestTagResult);
    vision.getBackLimelightTagResult().ifPresent(this::ingestTagResult);
    robotPose = drivetrain.getState().Pose;

    trustFactor.update(robotPose, imu.collisionDetected());
  }
}
