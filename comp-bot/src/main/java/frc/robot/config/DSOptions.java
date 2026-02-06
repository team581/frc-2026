package frc.robot.config;

import com.team581.config.DSOption;
import edu.wpi.first.networktables.BooleanSubscriber;

public final class DSOptions {
  public static final BooleanSubscriber SENSOR_OVERRIDE = DSOption.of("SensorOverride", false);
  public static final BooleanSubscriber USE_TAG_LIMELIGHTS = DSOption.of("UseTagLimelights", true);

  private DSOptions() {}
}
