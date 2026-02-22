package frc.robot.config;

import com.team581.autos.Point;
import com.team581.config.FeatureFlag;
import java.util.function.BooleanSupplier;

public class FeatureFlags {
  public static final BooleanSupplier VISION_HUB_TAGS_FILTER =
      FeatureFlag.of("OnlyUseHubTags", true);

  public static final BooleanSupplier BUMP_ASSIST = FeatureFlag.of("BumpAssist", true);
  public static final BooleanSupplier TRENCH_ASSIST = FeatureFlag.of("TrenchAssist", true);
  public static final BooleanSupplier WALL_INTAKE_DRIVE_ASSIST = FeatureFlag.of("WallIntakeDriveAssist", false);
  public static final BooleanSupplier HOPPER_SHUFFLING = FeatureFlag.of("HopperShuffling", true);
  public static final BooleanSupplier INTAKE_DIRECTIONAL_SNAPS =
      FeatureFlag.of("IntakeDirectionalSnaps", false);

  public static final BooleanSupplier INTEGRATION_TEST = FeatureFlag.of("IntegrationTest", false);

  public static final BooleanSupplier INTAKE_WALL_SNAPS = FeatureFlag.of("IntakeWallSnaps", false);

  public static final BooleanSupplier LOOKAHEAD_SCORING =
      FeatureFlag.of("TimeOfFlightInHubActivity", false);

  public static final BooleanSupplier RATE_LIMITED_DRIVING =
      FeatureFlag.of("RateLimitedDriving", true);

  public static final BooleanSupplier REGRESSION_MODEL = FeatureFlag.of("RegressionModel", false);

  public static final BooleanSupplier STOP_SHOOTING_STATE =
      FeatureFlag.of("UseStopShootingState", false);

  public static final BooleanSupplier CANCEL_IN_PROGRESS_SHOT_RPM_DIP =
      FeatureFlag.of("StopScoringRPMDip", true);
  public static final BooleanSupplier CANCEL_IN_PROGRESS_SHOT =
      FeatureFlag.of("CancelInProgressShot", true);

  public static final BooleanSupplier CLUSTER_MAP = FeatureFlag.of("ClusterMap", false);

  public static final BooleanSupplier CLAMPED_AUTO_POINTS = Point.CLAMPED_POINTS_FEATURE_FLAG;

  private FeatureFlags() {}
}
