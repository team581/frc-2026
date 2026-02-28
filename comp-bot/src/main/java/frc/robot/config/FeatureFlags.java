package frc.robot.config;

import com.team581.autos.Point;
import com.team581.config.FeatureFlag;
import java.util.function.BooleanSupplier;

public class FeatureFlags {

  public static final BooleanSupplier WALL_INTAKE_DRIVE_ASSIST =
      FeatureFlag.of("WallIntakeDriveAssist", false);
  public static final BooleanSupplier INTAKE_DIRECTIONAL_SNAPS =
      FeatureFlag.of("IntakeDirectionalSnaps", false);

  public static final BooleanSupplier INTEGRATION_TEST = FeatureFlag.of("IntegrationTest", false);

  public static final BooleanSupplier LOOKAHEAD_SCORING =
      FeatureFlag.of("TimeOfFlightInHubActivity", true);

  public static final BooleanSupplier REGRESSION_MODEL = FeatureFlag.of("RegressionModel", false);

  public static final BooleanSupplier STOP_SHOOTING_STATE =
      FeatureFlag.of("UseStopShootingState", false);

  public static final BooleanSupplier IGNORE_TURRET_AT_GOAL = FeatureFlag.of("IgnoreTurret", false);

  public static final BooleanSupplier CANCEL_IN_PROGRESS_SHOT =
      FeatureFlag.of("CancelInProgressShot", true);

  public static final BooleanSupplier HOOD_ALWAYS_IDLE = FeatureFlag.of("HoodAlwaysIdle", true);
  public static final BooleanSupplier DYE_ROTOR_CLEANUP_MODE =
      FeatureFlag.of("DyeRotorCleanup", false);

  public static final BooleanSupplier CLAMPED_AUTO_POINTS = Point.CLAMPED_POINTS_FEATURE_FLAG;

  private FeatureFlags() {}
}
