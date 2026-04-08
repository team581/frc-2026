package frc.robot.vision;

import com.team581.config.CameraConfig;
import com.team581.config.LimelightModel;
import edu.wpi.first.math.util.Units;

public class CameraConfigs {

  public static final CameraConfig SHOOTER =
      new CameraConfig(
          LimelightModel.FOUR,
          true,
          false,
          // Forward
          Units.inchesToMeters(-12.6378),
          // Right
          Units.inchesToMeters(-0.0002),
          // Up
          Units.inchesToMeters(16.0558),
          // Pitch
          20.0,
          // Yaw
          180.0,
          // Roll
          0.0);

  public static final CameraConfig LEFT =
      new CameraConfig(
          LimelightModel.FOUR,
          true,
          false,
          // Forward
          Units.inchesToMeters(-7.5115),
          // Right
          Units.inchesToMeters(-13.9087),
          // Up
          Units.inchesToMeters(16.5995),
          // Pitch
          0.0,
          // Yaw
          90.0,
          // Roll
          0.0);

  public static final CameraConfig RIGHT =
      new CameraConfig(
          LimelightModel.FOUR,
          true,
          false,
          // Forward
          Units.inchesToMeters(-7.5111),
          // Right
          Units.inchesToMeters(13.9087),
          // Up
          Units.inchesToMeters(16.5995),
          // Pitch
          0.0,
          // Yaw
          -90.0,
          // Roll
          0.0);

  public static final CameraConfig GROUND =
      new CameraConfig(
          LimelightModel.THREE,
          false,
          false,
          Units.inchesToMeters(25.054733),
          Units.inchesToMeters(0.0),
          Units.inchesToMeters(12.056034),
          -20.0,
          0.0,
          0.0);

  private CameraConfigs() {}
}
