package frc.robot.cluster_map;

import static java.util.Comparator.comparingDouble;

import com.team581.math.GamePieceDetectionCalculator;
import com.team581.math.MathHelpers;
import com.team581.util.state_machines.StateMachineSubsystem;
import com.team581.vision.limelight.LimelightHelpers;
import com.team581.vision.results.GamePieceResult;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.config.FeatureFlags;
import frc.robot.localization.Localization;
import frc.robot.swerve.Swerve;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.vision.limelight.Limelight;
import frc.robot.vision.limelight.LimelightState;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.Optional;

public class ClusterMap extends StateMachineSubsystem<ClusterMapState> {
  private static final double SAME_CLUSTER_DETECTION_THRESHOLD_METERS = 1.0;
  private static final double SWERVE_MAX_LINEAR_SPEED_TRACKING = 3.0;
  private static final double SWERVE_MAX_ANGULAR_SPEED_TRACKING = 3.0;

  private static final double CLUSTER_LIFETIME_SECONDS = 2;

  private final Limelight limelight;

  private final ArrayList<ClusterMapElement> clusterMap = new ArrayList<>();
  private double[] previousResult = new double[0];
  private boolean staleData = false;
  private ChassisSpeeds swerveSpeeds = new ChassisSpeeds();
  private Localization localization;
  private Swerve swerve;

  private final Comparator<Pose2d> bestClusterComparator =
      comparingDouble(
          target -> {
            return AlignmentCostUtil.getClusterAlignCost(
                target, localization.getPose(), swerve.getFieldRelativeSpeeds());
          });

  private final GamePieceResult gamePieceResult = new GamePieceResult();

  public ClusterMap(Localization localization, Swerve swerve, Limelight limelight) {
    super(SubsystemPriority.VISION, ClusterMapState.DEFAULT_STATE);
    this.localization = localization;
    this.swerve = swerve;
    this.limelight = limelight;
  }

  public Optional<Pose2d> getBestClusterPose() {
    if (RobotBase.isSimulation()) {
      var fakeBestCluster = new Pose2d(9, 5, localization.getPose().getRotation());
      var rotation = MathHelpers.getDriveDirection(localization.getPose(), fakeBestCluster);
      return timeout(5.0) ? Optional.of(new Pose2d(9, 5, rotation)) : Optional.empty();
    }
    if (clusterMap.isEmpty()) {
      return Optional.empty();
    }

    var bestCluster =
        clusterMap.stream()
            .map(cluster -> new Pose2d(cluster.clusterTranslation(), Rotation2d.kZero))
            .min(bestClusterComparator);
    if (bestCluster.isPresent()) {
      var rotation =
          MathHelpers.getDriveDirection(localization.getPose(), bestCluster.orElseThrow());
      if (!MathUtil.isNear(
          localization.getPose().getRotation().getDegrees(),
          rotation.getDegrees(),
          45,
          -180,
          180)) {
        return Optional.empty();
      }
      var clusterPoseWithIntakeRotation =
          new Pose2d(bestCluster.orElseThrow().getTranslation(), rotation);
      DogLog.log("ClusterMap/BestClusterPose", clusterPoseWithIntakeRotation);
      return Optional.of(clusterPoseWithIntakeRotation);
    }
    return Optional.empty();
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();
    try {
      DogLog.log(
          "ClusterMap/Clusters",
          clusterMap.stream()
              .map(element -> new Pose2d(element.clusterTranslation(), Rotation2d.kZero))
              .toArray(Pose2d[]::new));
    } catch (RuntimeException error) {
      DogLog.logFault("ClusterMapLoggingError");
      System.err.println(error);
    }
  }

  private Optional<Translation2d> getRawClusterPoses() {
    if (limelight.getState() != LimelightState.CLUSTER_MAP) {
      return Optional.empty();
    }

    double[] result = LimelightHelpers.getPythonScriptData(limelight.limelightTableName);

    if (result == null || result.length < 5) {
      DogLog.timestamp("ClusterMap/NoData");
      return Optional.empty();
    }
    // Check if the result array has changed
    staleData = Arrays.equals(previousResult, result);
    previousResult = result;
    if (staleData) {
      DogLog.timestamp("ClusterMap/SkipStaleData");

      return Optional.empty();
    }

    double latency =
        (LimelightHelpers.getLatency_Capture(limelight.limelightTableName)
                + LimelightHelpers.getLatency_Pipeline(limelight.limelightTableName))
            / 1000.0;

    double timestamp = Timer.getFPGATimestamp() - latency;

    var robotPoseAtCapture = localization.getPose(timestamp);

    double angleX = LimelightHelpers.getTX(limelight.limelightTableName);
    double angleY = LimelightHelpers.getTY(limelight.limelightTableName);

    gamePieceResult.update(angleX, angleY, timestamp);

    var clusterPose =
        GamePieceDetectionCalculator.calculateFieldRelativeTranslationFromCamera(
            robotPoseAtCapture, gamePieceResult, limelight.config);

    return Optional.of(clusterPose);
  }

  private boolean safeToTrack() {
    return swerveSpeeds.vxMetersPerSecond < SWERVE_MAX_LINEAR_SPEED_TRACKING
        && swerveSpeeds.vyMetersPerSecond < SWERVE_MAX_LINEAR_SPEED_TRACKING
        && swerveSpeeds.omegaRadiansPerSecond < Math.toRadians(SWERVE_MAX_ANGULAR_SPEED_TRACKING);
  }

  private void updateMap() {
    var latestPose = getRawClusterPoses();
    if (latestPose.isEmpty() || !safeToTrack()) {
      return;
    }

    var visionCluster = latestPose.orElseThrow();

    clusterMap.removeIf(
        element -> {
          return (element.expiresAt() < Timer.getFPGATimestamp());
        });

    if (staleData) {
      return;
    }

    double newClusterExpiry = Timer.getFPGATimestamp() + CLUSTER_LIFETIME_SECONDS;

    Optional<ClusterMapElement> match =
        clusterMap.stream()
            .filter(
                rememberedCluster -> {
                  return rememberedCluster.expiresAt() != newClusterExpiry
                      && (rememberedCluster.clusterTranslation().getDistance(visionCluster)
                          < SAME_CLUSTER_DETECTION_THRESHOLD_METERS);
                })
            .min(
                (a, b) ->
                    Double.compare(
                        a.clusterTranslation().getDistance(visionCluster),
                        b.clusterTranslation().getDistance(visionCluster)));

    if (match.isPresent()) {
      var existingCluster = match.orElseThrow().clusterTranslation();
      var health = match.orElseThrow().health() + 1;
      var blendedPose = existingCluster.interpolate(visionCluster, 1 / health);
      clusterMap.add(new ClusterMapElement(newClusterExpiry, blendedPose, health));
      clusterMap.remove(match.orElseThrow());
    } else {
      clusterMap.add(new ClusterMapElement(newClusterExpiry, visionCluster, 1.0));
    }
  }

  @Override
  protected void collectInputs() {
    if (!FeatureFlags.CLUSTER_MAP.getAsBoolean()) {
      return;
    }
    swerveSpeeds = swerve.getRobotRelativeSpeeds();
    updateMap();
  }
}
