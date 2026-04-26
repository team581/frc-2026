package frc.robot.config;

import com.team581.config.DSOption;
import edu.wpi.first.networktables.BooleanSubscriber;

public final class DSOptions {
  public static final BooleanSubscriber USE_TAG_LIMELIGHTS = DSOption.of("UseTagLimelights", true);
  public static final BooleanSubscriber USE_HUB_STATE = DSOption.of("UseHubState", true);
  public static final BooleanSubscriber USE_BUMP_ASSIST = DSOption.of("UseBumpAssist", true);
  public static final BooleanSubscriber USE_CANRANGE = DSOption.of("UseCANRange", true);
  public static final BooleanSubscriber DEFAULT_WON_AUTO = DSOption.of("DefaultWonAuto", true);
  public static final BooleanSubscriber RESET_POSE_FOR_AUTO = DSOption.of("ResetPoseForAuto", true);
  public static final BooleanSubscriber USE_TELEOP_TIMER = DSOption.of("UseTeleopTimer", true);

  public static final BooleanSubscriber PIT_FUNCTIONALITY = DSOption.of("PitFunctionality", false);

  private DSOptions() {}
}
