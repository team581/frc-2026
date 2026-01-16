package frc.robot.robot_manager;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public final class MechanismVisualizer {
  /** Height of the turret pivot point from the floor. */
  private static final double TURRET_HEIGHT_METERS = Units.inchesToMeters(24);

  public static void log(Pose2d robotPose, double turretAngleDegrees) {
    // Transform from robot center to turret, including height and turret rotation
    var turretTransform =
        new Transform3d(
            new Translation3d(0, 0, TURRET_HEIGHT_METERS),
            new Rotation3d(0, 0, Units.degreesToRadians(turretAngleDegrees)));

    // Convert robot pose to 3D and apply the turret transform
    var robotPose3d = new Pose3d(robotPose);
    var turretPose = robotPose3d.transformBy(turretTransform);

    // Add this as a Pose3d displayed as a cone in the 3D field view
    // Or as an arrow in the 2D field view
    DogLog.log("Turret/Pose3d", turretPose);
  }

  private MechanismVisualizer() {}
}
