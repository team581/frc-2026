package frc.robot.robot_manager;

import com.team581.simkit.SimKit;
import com.team581.simkit.internal.SimShooter;
import com.team581.util.FieldUtil;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.shooter_hood.ShooterHoodConfig;
import frc.robot.turret.TurretConfig;
import frc.robot.vision.CameraConfigs;
import frc.robot.vision.VisionConfig;
import org.jspecify.annotations.Nullable;

public final class MechanismVisualizer {
  /**
   * If (0, 0, 0) is the robot origin, this translation defines the point that the shooter hood
   * pivots around.
   */
  static final Translation3d SHOOTER_HOOD_PIVOT_POINT = new Translation3d(0.118364, 0, 0.436753);

  /** Angle from the horizontal to the deploy extension point. */
  private static final double DEPLOY_ANGLE_FROM_HORIZONTAL = 15.327113;

  private static final double GRAVITY = 9.81;

  /**
   * Effective wheel radius in meters, used to convert shooter RPM to ball exit velocity. This
   * accounts for ball compression and slip, so it will likely differ from the physical wheel
   * radius. Tune this to match observed shot distances.
   */
  private static final DoubleSubscriber EFFECTIVE_WHEEL_RADIUS =
      (DogLog.tunable("SimShooter/EffectiveWheelRadiusInches", 1.4));

  /**
   * Additive offset in degrees applied to the hood angle to account for the difference between the
   * mechanical hood angle and the actual ball exit angle. Tune this to match observed shot arcs.
   */
  private static final DoubleSubscriber LAUNCH_ANGLE_OFFSET_DEGREES =
      DogLog.tunable("SimShooter/LaunchAngleOffsetDegrees", 14.0);

  private static @Nullable SimShooter simShooter;

  /** Timestamp of the last simulated shot, used for rate limiting by BPS. */
  private static double lastShotTimestamp = 0;

  /**
   * Computes the field-relative 3D position of the shooter exit point.
   *
   * @param robotPose The robot's field-relative pose.
   * @param turretAngleDegrees The turret's current angle in degrees.
   * @return The field-relative 3D position of the shooter exit.
   */
  public static Translation3d getShooterExitPoint(Pose2d robotPose, double turretAngleDegrees) {
    var robotPose3d = new Pose3d(robotPose);
    var shooterPose =
        robotPose3d.plus(
            new Transform3d(
                new Translation3d(TurretConfig.TURRET_TO_ROBOT.getX(), 0, 0),
                new Rotation3d(0, 0, Math.toRadians(turretAngleDegrees))));
    return new Translation3d(
        shooterPose.getX(), shooterPose.getY(), SHOOTER_HOOD_PIVOT_POINT.getZ());
  }

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

    var fuelTrajectory = getSimShooter();
    if (fuelTrajectory != null) {
      fuelTrajectory.update();
    }
  }

  /**
   * Checks if fuel is being ejected and spawns simulated shots at the appropriate rate. The
   * trajectory is computed purely from physics: exit velocity (from shooter RPM and wheel radius)
   * and launch direction (from hood pitch and turret yaw).
   *
   * @param robotPose The robot's field-relative pose.
   * @param turretAngleDegrees The turret's current angle in degrees.
   * @param shooterHoodAngleDegrees The hood's current angle from horizontal in degrees.
   * @param shooterRpm The shooter's current average RPM.
   * @param isDyeRotorShooting Whether the dye rotor is actively ejecting fuel.
   * @param bps Balls per second being ejected by the dye rotor.
   */
  public static void updateShotSimulation(
      Pose2d robotPose,
      double turretAngleDegrees,
      double shooterHoodAngleDegrees,
      double shooterRpm,
      boolean isDyeRotorShooting,
      double bps) {
    var fuelTrajectory = getSimShooter();
    if (fuelTrajectory == null) {
      return;
    }

    if (!isDyeRotorShooting || bps <= 0 || shooterRpm <= 0) {
      return;
    }

    double now = Timer.getFPGATimestamp();
    double shotInterval = 1.0 / bps;

    if (now - lastShotTimestamp >= shotInterval) {
      var start = getShooterExitPoint(robotPose, turretAngleDegrees);

      // Exit speed from shooter RPM and effective wheel radius
      var exitSpeed =
          shooterRpm * 2.0 * Math.PI * Units.inchesToMeters(EFFECTIVE_WHEEL_RADIUS.get()) / 60.0;

      // Launch angle from hood position plus tunable offset
      var launchAngleRad =
          Math.toRadians(shooterHoodAngleDegrees + LAUNCH_ANGLE_OFFSET_DEGREES.get());

      // Field-relative turret yaw
      var turretYawRad = robotPose.getRotation().getRadians() + Math.toRadians(turretAngleDegrees);

      // Decompose exit velocity into field-relative components
      var vHorizontal = exitSpeed * Math.cos(launchAngleRad);
      var vz = exitSpeed * Math.sin(launchAngleRad);
      var vx = vHorizontal * Math.cos(turretYawRad);
      var vy = vHorizontal * Math.sin(turretYawRad);

      // Time to hit z=0: start.z + vz*t - 0.5*g*t^2 = 0
      // Quadratic formula: t = (vz + sqrt(vz^2 + 2*g*z0)) / g
      var z0 = start.getZ();
      var discriminant = vz * vz + 2.0 * GRAVITY * z0;
      if (discriminant < 0) {
        return;
      }
      var tofSeconds = (vz + Math.sqrt(discriminant)) / GRAVITY;

      if (tofSeconds <= 0) {
        return;
      }

      // Landing point at z=0
      var target =
          new Translation3d(start.getX() + vx * tofSeconds, start.getY() + vy * tofSeconds, 0);

      fuelTrajectory.shoot(start, target, tofSeconds);
      lastShotTimestamp = now;
    }
  }

  private static SimShooter getSimShooter() {
    if (simShooter == null) {
      var redHub = FieldUtil.HUB_POSE.redPose().getTranslation();
      var blueHub = FieldUtil.HUB_POSE.bluePose().getTranslation();
      var hubRadius = FieldUtil.HUB_RADIUS_METERS;

      simShooter =
          SimKit.shooter(
              "fuelTrajectory",
              builder ->
                  builder
                      .withLogKey("SuperstructureVisualization/Fuel")
                      .withMinZ(0 - Units.inchesToMeters(6))
                      .withMaxGpLifetime(5)
                      .withRemovalFunction(
                          shot -> {
                            var pos = shot.pose();
                            var xy = new Translation2d(pos.getX(), pos.getY());
                            var inHub =
                                xy.getDistance(redHub) < hubRadius
                                    || xy.getDistance(blueHub) < hubRadius;
                            return inHub && pos.getZ() < Units.inchesToMeters(72 - 6);
                          }));
    }
    return simShooter;
  }

  private MechanismVisualizer() {}
}
