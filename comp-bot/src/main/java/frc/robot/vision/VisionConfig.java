package frc.robot.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;

public class VisionConfig {
  // Turret pose relative to camera
  public static final Transform2d TURRET_TO_CAMERA =
      new Transform2d(Units.inchesToMeters(6.819491), 0.0, Rotation2d.kZero);

  private VisionConfig() {}
}
