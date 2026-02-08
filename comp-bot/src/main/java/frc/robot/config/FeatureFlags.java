package frc.robot.config;

import com.team581.config.FeatureFlag;
import java.util.function.BooleanSupplier;

public class FeatureFlags {
  public static final BooleanSupplier VISION_HUB_TAGS_FILTER =
      FeatureFlag.of("OnlyUseHubTags", true);

  public static final BooleanSupplier BUMP_ASSIST = FeatureFlag.of("BumpAssist", true);
  public static final BooleanSupplier TRENCH_ASSIST = FeatureFlag.of("TrenchAssist", true);
  public static final BooleanSupplier HOPPER_SHUFFLING = FeatureFlag.of("HopperShuffling", true);
  public static final BooleanSupplier INTAKE_DIRECTIONAL_SNAPS =
      FeatureFlag.of("IntakeDirectionalSnaps", false);

  public static final BooleanSupplier INTAKE_WALL_SNAPS = FeatureFlag.of("IntakeWallSnaps", false);

  public static final BooleanSupplier LOOKAHEAD_SCORING =
      FeatureFlag.of("TimeOfFlightInHubActivity", false);

  public static final BooleanSupplier REGRESSION_MODEL = FeatureFlag.of("RegressionModel", false);

  private FeatureFlags() {}
}
