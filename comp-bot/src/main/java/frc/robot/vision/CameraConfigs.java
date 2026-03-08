package frc.robot.vision;

import com.team581.config.CameraConfig;
import com.team581.config.LimelightModel;
import edu.wpi.first.math.util.Units;

public class CameraConfigs {
  public static final CameraConfig TURRET =
      new CameraConfig(
          LimelightModel.FOUR,
          true,
          false,
          Units.inchesToMeters(0.0),
          Units.inchesToMeters(0.0),
          Units.inchesToMeters(21.0309),
          30.0,
          0.0,
          0.0);

  public static final CameraConfig BACK =
      new CameraConfig(
          LimelightModel.FOUR,
          true,
          true,
          // back
          Units.inchesToMeters(-13.389),
          // left
          Units.inchesToMeters(-8.3370),
          Units.inchesToMeters(19.7564),
          // TODO: get real number from cad
          10.00,
          -175.5,
          -0.83);

  // ground when stowed
  // Units.inchesToMeters(12.9742),
  // Units.inchesToMeters(0.0),
  // Units.inchesToMeters(16.8886)

  public static final CameraConfig GROUND =
      new CameraConfig(
          LimelightModel.THREE,
          false,
          false,
          Units.inchesToMeters(25.671),
          Units.inchesToMeters(0.0),
          Units.inchesToMeters(12.9525),
          -20.0,
          0.0,
          0.0);

  private CameraConfigs() {}
}
