package frc.robot.robot_manager;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.shooter_hood.ShooterHoodConfig;
import frc.robot.turret.TurretConfig;
import frc.robot.vision.CameraConfigs;
import frc.robot.vision.VisionConfig;

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
      Pose2d robotPose,
      double turretAngleDegrees,
      double shooterHoodAngleDegrees,
      double deployLengthInches,
      double climberHeightInches,
      double dyeRotorAngleDegrees) {
    var turretPose =
        new Pose3d(Translation3d.kZero, new Rotation3d(Rotation2d.fromDegrees(turretAngleDegrees)));
    var shooterHoodPose =
        Pose3d.kZero
            .rotateAround(
                SHOOTER_HOOD_PIVOT_POINT,
                new Rotation3d(
                    0,
                    Math.toRadians(
                        shooterHoodAngleDegrees - ShooterHoodConfig.ANGLE_FROM_HORIZONTAL),
                    0))
            .rotateBy(turretPose.getRotation());
    var deployPose =
        new Pose3d(
            new Translation3d(Units.inchesToMeters(deployLengthInches), 0, 0)
                .rotateBy(new Rotation3d(0, Math.toRadians(DEPLOY_ANGLE_FROM_HORIZONTAL), 0)),
            Rotation3d.kZero);
    var climberPose =
        new Pose3d(
            new Translation3d(0, 0, Units.inchesToMeters(climberHeightInches)), Rotation3d.kZero);
    var dyeRotorPose =
        new Pose3d(
            new Translation3d(0, 0, 0),
            new Rotation3d(Rotation2d.fromDegrees(-dyeRotorAngleDegrees)));

    DogLog.log(
        "SuperstructureVisualization/Components",
        new Pose3d[] {turretPose, shooterHoodPose, deployPose, climberPose, dyeRotorPose});

    // Field-relative turret camera pose for AdvantageScope Camera Override
    var cameraTransform = CameraConfigs.TURRET.getTransform3d();
    var cameraRotation = cameraTransform.getRotation();
    var turretCameraPose =
        new Pose3d(robotPose)
            // Robot center to turret pivot + turret rotation
            .plus(
                new Transform3d(
                    new Translation3d(TurretConfig.TURRET_TO_ROBOT.getX(), 0, 0),
                    new Rotation3d(0, 0, Math.toRadians(turretAngleDegrees))))
            // Turret pivot to camera
            .plus(
                new Transform3d(
                    new Translation3d(VisionConfig.TURRET_TO_CAMERA.getX(), 0, 0),
                    Rotation3d.kZero))
            // Camera height + orientation (pitch negated for AdvantageScope convention)
            .plus(
                new Transform3d(
                    cameraTransform.getTranslation(),
                    new Rotation3d(
                        cameraRotation.getX(), -cameraRotation.getY(), cameraRotation.getZ())));
    DogLog.log("Vision/TurretCameraOverride", turretCameraPose);
  }

  private MechanismVisualizer() {}
}
