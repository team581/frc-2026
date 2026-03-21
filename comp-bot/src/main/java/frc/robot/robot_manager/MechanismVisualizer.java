package frc.robot.robot_manager;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.shooter_hood.ShooterHoodConfig;

public final class MechanismVisualizer {
  /**
   * If (0, 0, 0) is the robot origin, this translation defines the point that the shooter hood
   * pivots around.
   */
  private static final Translation3d SHOOTER_HOOD_PIVOT_POINT =
      new Translation3d(0.118364, 0, 0.436753);

  /** Angle from the horizontal to the deploy extension point. */
  private static final double DEPLOY_ANGLE_FROM_HORIZONTAL = 15.327113;

  public static void log(
      Pose2d robotPose, double shooterHoodAngleDegrees, double deployLengthInches) {
    var shooterHoodPose =
        Pose3d.kZero.rotateAround(
            SHOOTER_HOOD_PIVOT_POINT,
            new Rotation3d(
                0,
                Math.toRadians(shooterHoodAngleDegrees - ShooterHoodConfig.ANGLE_FROM_HORIZONTAL),
                0));
    var deployPose =
        new Pose3d(
            new Translation3d(Units.inchesToMeters(deployLengthInches), 0, 0)
                .rotateBy(new Rotation3d(0, Math.toRadians(DEPLOY_ANGLE_FROM_HORIZONTAL), 0)),
            Rotation3d.kZero);

    DogLog.log(
        "SuperstructureVisualization/Components", new Pose3d[] {shooterHoodPose, deployPose});
  }

  private MechanismVisualizer() {}
}
