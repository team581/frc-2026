package frc.robot.vision.limelight;

import com.team581.config.CameraConfig;
import com.team581.config.LimelightModel;
import com.team581.mechanisms.vision.CameraHealth;
import com.team581.util.ReusableOptional;
import com.team581.util.state_machines.StateMachineSubsystem;
import com.team581.vision.results.OptionalTagResult;
import dev.doglog.DogLog;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.scheduling.SubsystemPriority;
import java.util.Locale;

public class Limelight extends StateMachineSubsystem<LimelightState> {
  private static final int[] VALID_APRILTAGS =
      new int[] {
        1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25,
        26, 27, 28, 29, 30, 31, 32
      };

  private static final int[] HUB_TAGS = new int[] {2, 3, 4, 5, 8, 9, 10, 11};

  private static final double IS_OFFLINE_TIMEOUT = 3;

  private final String limelightTableName;
  private final String name;
  private final CameraConfig config;

  private final Timer limelightTimer = new Timer();
  private final Timer seedImuTimer = new Timer();
  private CameraHealth cameraHealth = CameraHealth.NO_TARGETS;
  private double limelightHeartbeat = -1;

  private OptionalTagResult lastGoodTagResult = new OptionalTagResult();
  private OptionalTagResult tagResult = new OptionalTagResult();

  private double angularVelocity = 0.0;
  private boolean updatedLimelightPos = false;

  public Limelight(String name, LimelightState initialState, CameraConfig config) {
    super(SubsystemPriority.VISION, initialState);
    limelightTableName = "limelight-" + name;
    this.name = name;
    limelightTimer.start();
    this.config = config;
  }

  public void sendImuData(
      double robotHeading,
      double angularVelocity,
      double pitch,
      double pitchRate,
      double roll,
      double rollRate) {
    LimelightHelpers.SetRobotOrientation(
        limelightTableName, robotHeading, angularVelocity, pitch, pitchRate, roll, rollRate);
    this.angularVelocity = angularVelocity;
  }

  public void setState(LimelightState state) {
    setStateFromRequest(state);
  }

  public OptionalTagResult getTagResult() {
    if (getState() != LimelightState.TAGS && getState() != LimelightState.HUB_TAGS) {
      DogLog.log("Vision/" + name + "/Tags/RawLimelightPose", Pose2d.kZero);
      return tagResult.empty();
    }

    var mTEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightTableName);

    if (mTEstimate == null) {
      return tagResult.empty();
    }

    var mTEstimateTimestamp = mTEstimate.timestampSeconds;

    // TODO(@simonstoryparker): Make work with angular velocity from swerve + turret
    if (Math.abs(angularVelocity) > 360) {
      return tagResult.empty();
    }
    if (mTEstimate.tagCount == 0) {
      DogLog.log("Vision/" + name + "/Tags/RawLimelightPose", Pose2d.kZero);

      return tagResult.empty();
    }
    if (mTEstimate.rawFiducials.length == 1) {
      double ambiguity = mTEstimate.rawFiducials[0].ambiguity;
      if (ambiguity >= 0.7) {
        DogLog.timestamp("Vision/" + name + "/Tags/AmbiguityFilter");
        return tagResult.empty();
      }
    }

    var mtPose = mTEstimate.pose;

    // This prevents pose estimator from having crazy poses if the Limelight loses power
    if (mtPose.getX() == 0.0 && mtPose.getY() == 0.0) {
      DogLog.log("Vision/" + name + "/Tags/RawLimelightPose", Pose2d.kZero);
      return tagResult.empty();
    }

    var distance = mTEstimate.avgTagDist;
    var xyDev = 0.01 * Math.pow(distance, 1.2);
    var thetaDev = 0.03 * Math.pow(distance, 1.2);

    if (distance > Units.inchesToMeters(40) || !config.useMegatag1RotationWhenClose()) {
      thetaDev = Double.POSITIVE_INFINITY;
    }

    var devs = VecBuilder.fill(xyDev, xyDev, thetaDev);

    DogLog.log("Vision/" + name + "/Tags/RawLimelightPose", mtPose);
    DogLog.log("Vision/" + name + "/Tags/MT2Timestamp", mTEstimateTimestamp);
    DogLog.log("Vision/" + name + "/Tags/DistanceFromTag", distance);
    return tagResult.update(mtPose, mTEstimateTimestamp, devs);
  }

  @Override
  protected void collectInputs() {
    tagResult = getTagResult();
    if (tagResult.isPresent()) {
      lastGoodTagResult = tagResult;
    }
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();

    if (DriverStation.isDisabled()) {
      if (!updatedLimelightPos && getCameraHealth() != CameraHealth.OFFLINE) {
        LimelightHelpers.setCameraPose_RobotSpace(
            limelightTableName,
            config.forward(),
            config.right(),
            config.up(),
            config.roll(),
            config.pitch(),
            config.yaw());

        updatedLimelightPos = true;
      }
      if (config.model() == LimelightModel.FOUR) {
        LimelightHelpers.SetThrottle(limelightTableName, 10);
      }
    } else {
      LimelightHelpers.SetThrottle(limelightTableName, 0);
    }
    DogLog.log("Vision/" + name + "/State", getState());

    var lastTagTimestamp =
        lastGoodTagResult.isPresent()
            ? lastGoodTagResult.orElseThrow().timestamp()
            : Double.MIN_VALUE;

    if (Timer.getTimestamp() - lastTagTimestamp > 30) {
      DogLog.logFault(
          limelightTableName + " has not seen a tag in the last 30 seconds", AlertType.kWarning);
    } else {
      DogLog.clearFault(limelightTableName + " has not seen a tag in the last 30 seconds");
    }

    LimelightHelpers.setPipelineIndex(limelightTableName, getState().pipelineIndex);
    switch (getState()) {
      case TAGS -> {
        if (limelightTimer.hasElapsed(5.0)) {
          LimelightHelpers.SetFiducialIDFiltersOverride(limelightTableName, VALID_APRILTAGS);
        }
        updateHealth(tagResult);
      }
      case HUB_TAGS -> {
        if (limelightTimer.hasElapsed(5.0)) {
          LimelightHelpers.SetFiducialIDFiltersOverride(limelightTableName, HUB_TAGS);
        }
        updateHealth(tagResult);
      }
      default -> {}
    }

    LimelightHelpers.SetIMUMode(limelightTableName, 0);
  }

  @Override
  public void autonomousInit() {
    if (config.model() != LimelightModel.THREE) {
      LimelightHelpers.SetFiducialIDFiltersOverride(limelightTableName, VALID_APRILTAGS);
    }
    seedImuTimer.reset();
    seedImuTimer.start();
  }

  @Override
  public void teleopInit() {
    if (config.model() != LimelightModel.THREE) {
      LimelightHelpers.SetFiducialIDFiltersOverride(limelightTableName, VALID_APRILTAGS);
    }
  }

  private void updateHealth(ReusableOptional<?> result) {
    var newHeartbeat = LimelightHelpers.getHeartbeat(limelightTableName);
    DogLog.log("Vision/" + name + "/Heartbeat", newHeartbeat);
    if (limelightHeartbeat != newHeartbeat) {
      limelightTimer.restart();
    }
    limelightHeartbeat = newHeartbeat;

    if (limelightTimer.hasElapsed(IS_OFFLINE_TIMEOUT) && RobotBase.isReal()) {
      cameraHealth = CameraHealth.OFFLINE;
      DogLog.logFault(name.toUpperCase(Locale.US) + "LIMELIGHT IS OFFLINE", AlertType.kError);
      return;
    } else {
      DogLog.clearFault(name.toUpperCase(Locale.US) + "LIMELIGHT IS OFFLINE");
    }

    if (result.isPresent()) {
      cameraHealth = CameraHealth.GOOD;
      return;
    }
    cameraHealth = CameraHealth.NO_TARGETS;
  }

  public void setBlinkEnabled(boolean enabled) {
    if (enabled) {
      LimelightHelpers.setLEDMode_ForceBlink(limelightTableName);
    } else {
      LimelightHelpers.setLEDMode_ForceOff(limelightTableName);
    }
  }

  public CameraHealth getCameraHealth() {
    DogLog.log("Vision/" + name + "/Health", cameraHealth);
    return cameraHealth;
  }

  public boolean isOnlineForTags() {
    return switch (getState()) {
      case TAGS, HUB_TAGS, OFF -> getCameraHealth() != CameraHealth.OFFLINE;
      default -> false;
    };
  }
}
