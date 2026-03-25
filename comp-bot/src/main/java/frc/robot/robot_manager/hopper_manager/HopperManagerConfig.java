package frc.robot.robot_manager.hopper_manager;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;

public class HopperManagerConfig {
  public static final CANrangeConfiguration CAN_RANGE_CONFIG = new CANrangeConfiguration();
  public static final DoubleSubscriber HOPPER_COMPACTION_DELAY =
      DogLog.tunable("HopperManager/CompactionDelay", 0.0);
}
