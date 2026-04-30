package frc.robot;

import com.team581.Base581Robot;
import com.team581.GlobalConfig;
import com.team581.controller.ControllerBindings;
import com.team581.math.PoseErrorTolerance;
import com.team581.trailblazer.Trailblazer;
import com.team581.trailblazer.followers.PidPathFollower;
import com.team581.trailblazer.trackers.HeuristicPathTracker;
import com.team581.util.FieldUtil;
import com.team581.util.FmsUtil;
import dev.doglog.DogLog;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.autos.Autos;
import frc.robot.autos.BumpCrossingFollower;
import frc.robot.cluster_map.ClusterMap;
import frc.robot.config.FeatureFlags;
import frc.robot.conveyor.Conveyor;
import frc.robot.deploy.Deploy;
import frc.robot.feeder.Feeder;
import frc.robot.generated.BuildConstants;
import frc.robot.health.HealthManager;
import frc.robot.hub_activity.HubActivity;
import frc.robot.imu.Imu;
import frc.robot.intake.Intake;
import frc.robot.localization.Localization;
import frc.robot.power_manager.PowerManager;
import frc.robot.robot_manager.RobotManager;
import frc.robot.robot_manager.hopper_manager.HopperManager;
import frc.robot.shooter.Shooter;
import frc.robot.shooter_hood.ShooterHood;
import frc.robot.swerve.Swerve;
import frc.robot.vision.CameraConfigs;
import frc.robot.vision.Vision;
import frc.robot.vision.limelight.Limelight;
import frc.robot.vision.limelight.LimelightState;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;

public class Robot extends Base581Robot {
  private final Hardware hardware = new Hardware();

  private final Limelight shooterLimelight =
      new Limelight("shooter", LimelightState.TAGS, CameraConfigs.SHOOTER);
  private final Limelight leftLimelight =
      new Limelight("left", LimelightState.TAGS, CameraConfigs.LEFT);
  private final Limelight rightLimelight =
      new Limelight("right", LimelightState.TAGS, CameraConfigs.RIGHT);
  private final Limelight groundLimelight =
      new Limelight("ground", LimelightState.CLUSTER_MAP, CameraConfigs.GROUND);
  private final HealthManager health =
      new HealthManager(shooterLimelight, leftLimelight, rightLimelight, groundLimelight);

  private final Imu imu = new Imu(hardware.drivetrain);

  private final Trailblazer trailblazer =
      new Trailblazer(
          new HeuristicPathTracker(new PoseErrorTolerance(0.5, 10)),
          new BumpCrossingFollower(
              new PidPathFollower(
                  new PIDController(3.5, 0, 0),
                  new PIDController(
                      Swerve.ORIGINAL_HEADING_PID.getP(),
                      Swerve.ORIGINAL_HEADING_PID.getI(),
                      Swerve.ORIGINAL_HEADING_PID.getD())),
              imu.bumpCrossingTracker));

  private final Swerve swerve =
      new Swerve(hardware.drivetrain, health, hardware.driverController, trailblazer);

  private final ShooterHood shooterHood = new ShooterHood(hardware.shooterHoodMotor);

  private final Shooter shooter =
      new Shooter(
          hardware.shooterTopLeftMotor,
          hardware.shooterTopRightMotor,
          hardware.shooterBottomLeftMotor,
          hardware.shooterBottomRightMotor);
  private final Intake intake = new Intake(hardware.intakeLeftMotor, hardware.intakeRightMotor);
  private final Deploy deploy = new Deploy(hardware.deployDifferentialMechanism);
  private final Vision vision =
      new Vision(imu, shooterLimelight, leftLimelight, rightLimelight, groundLimelight);
  private final Localization localization =
      new Localization(swerve, hardware.drivetrain, vision, imu);
  private final Feeder feeder = new Feeder(hardware.feederTopMotor, hardware.feederBottomMotor);
  private final Conveyor conveyor =
      new Conveyor(hardware.conveyorTopMotor, hardware.conveyorBottomMotor);

  private final ClusterMap clusterMap = new ClusterMap(localization, swerve, groundLimelight);
  private final HubActivity hubActivity = new HubActivity();

  private final PowerManager powerManager =
      new PowerManager(shooter, intake, deploy, shooterHood, feeder, conveyor, swerve);
  private final HopperManager hopperManager =
      new HopperManager(
          deploy, intake, conveyor, feeder, hardware.hopperCANRange, hardware.towerSensor);

  private final RobotManager robotManager =
      new RobotManager(
          hopperManager,
          shooterHood,
          localization,
          swerve,
          shooter,
          vision,
          hardware.driverController,
          health,
          hubActivity,
          trailblazer,
          clusterMap,
          hardware,
          powerManager);

  @SuppressWarnings("unused") // Registers itself as a subsystem
  private final Autos autos = new Autos(robotManager, trailblazer);

  public Robot() {
    logMetadata(
        BuildConstants.MAVEN_NAME,
        BuildConstants.BUILD_DATE,
        BuildConstants.GIT_SHA,
        BuildConstants.GIT_DATE,
        BuildConstants.GIT_BRANCH,
        BuildConstants.DIRTY);

    // Reuse the SwerveDriveState from Swerve for IMU
    imu.setDriveStateSupplier(swerve::getDriveState);

    finalizeInit();

    if (GlobalConfig.IS_DEVELOPMENT) {
      FieldUtil.debugLogFieldZones();
    }

    if (RobotBase.isSimulation()) {
      try {
        var docsDir = Path.of(System.getProperty("user.dir")).resolve("../docs");
        Files.writeString(
            docsDir.resolve("feeding_obstructions.svg"), FieldUtil.FEEDING_OBSTRUCTIONS.toSvg());
      } catch (IOException e) {
        throw new RuntimeException("Failed to write field obstacles SVG", e);
      }
    }
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();

    if (FeatureFlags.CLAMPED_AUTO_POINTS.getAsBoolean() && !FmsUtil.isRedAlliance()) {
      DogLog.logFault("Clamped auto points are enabled but current alliance is blue");
    } else {
      DogLog.clearFault("Clamped auto points are enabled but current alliance is blue");
    }
  }

  @Override
  protected void configureBindings() {
    var driver =
        new ControllerBindings(buttonBindingsLoop, enabledEvent, hardware.driverController);
    var operator =
        new ControllerBindings(buttonBindingsLoop, enabledEvent, hardware.operatorController);

    driver.back().onPress(localization::zeroGyro);

    driver
        .leftTrigger()
        .onPress(() -> hopperManager.setDriverWantsIntake(true))
        .onRelease(() -> hopperManager.setDriverWantsIntake(false));

    driver
        .rightTrigger()
        .onPress(robotManager::prepareScoreOrFeedRequest)
        .onRelease(robotManager::idleRequest);

    driver.rightBumper().onPress(robotManager::idleRequest);

    driver
        .leftBumper()
        .onPress(() -> hopperManager.setDriverWantsEject(true))
        .onRelease(() -> hopperManager.setDriverWantsEject(false));

    operator.start().onPress(() -> hopperManager.deploy.homingRequest());

    operator.back().onPress(robotManager::homeShooterHoodRequest);

    operator.x().onPress(robotManager::unjamRequest).onRelease(robotManager::idleRequest);

    operator
        .y()
        .onPress(
            () -> {
              powerManager.turboRequest();
              shooter.setTurboMode(true);
            })
        .onRelease(
            () -> {
              powerManager.idleRequest();
              shooter.setTurboMode(false);
            });

    // operator.b().onPress(robotManager::prepareFeedRequest).onRelease(robotManager::idleRequest);

    operator.b().onPress(deploy::fixDifferentialDesyncRequest).onRelease(deploy::intakeRequest);

    operator
        .rightTrigger()
        .onPress(robotManager::prepareScoreRequest)
        .onRelease(robotManager::idleRequest);

    operator
        .leftTrigger()
        .onPress(robotManager::stowDeployRequest)
        .onRelease(robotManager::cancelStowDeployRequest);

    operator
        .leftBumper()
        .onPress(() -> robotManager.powerManager.prioritizeIntakeRequest())
        .onRelease(() -> robotManager.powerManager.idleRequest());
    operator
        .rightBumper()
        .onPress(() -> robotManager.setTrenchOverrideRequest(true))
        .onRelease(() -> robotManager.setTrenchOverrideRequest(false));
  }
}
