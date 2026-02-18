package frc.robot.config;

import com.team581.config.DSOption;
import edu.wpi.first.networktables.BooleanSubscriber;

public final class DSOptions {
  public static final BooleanSubscriber USE_TAG_LIMELIGHTS = DSOption.of("UseTagLimelights", true);
  public static final BooleanSubscriber USE_HUB_STATE = DSOption.of("UseHubState", false);
  // TODO: This should be true when the CANrange is actually physically on the robot
  public static final BooleanSubscriber USE_CANRANGE = DSOption.of("UseCANRange", false);

  private DSOptions() {}
}
