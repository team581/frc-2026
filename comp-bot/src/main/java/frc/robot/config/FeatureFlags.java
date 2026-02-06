package frc.robot.config;

import com.team581.config.FeatureFlag;
import java.util.function.BooleanSupplier;

public class FeatureFlags {
  public static final BooleanSupplier VISION_HUB_TAGS_FILTER =
      FeatureFlag.of("OnlyUseHubTags", true);

  public static final BooleanSupplier DO_AUTO_SAFE_CHECK =
      FeatureFlag.of("RequireBeenInAuto", false);

  private FeatureFlags() {}
}
