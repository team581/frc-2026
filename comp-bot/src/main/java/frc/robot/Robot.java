package frc.robot;

import com.team581.Base581Robot;
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
import frc.robot.climber.Climber;
import frc.robot.cluster_map.ClusterMap;
import frc.robot.config.FeatureFlags;
import frc.robot.config.RobotKind;
import frc.robot.deploy.Deploy;
import frc.robot.dye_rotor.DyeRotor;
import frc.robot.generated.BuildConstants;
import frc.robot.health.HealthManager;
import frc.robot.hub_activity.HubActivity;
import frc.robot.imu.Imu;
import frc.robot.intake.GenericIntake;
import frc.robot.intake.Intake;
import frc.robot.intake.IntakeTwoMotor;
import frc.robot.localization.Localization;
import frc.robot.robot_manager.RobotManager;
import frc.robot.shooter.Shooter;
import frc.robot.shooter_hood.ShooterHood;
import frc.robot.swerve.Swerve;
import frc.robot.turret.Turret;
import frc.robot.vision.CameraConfigs;
import frc.robot.vision.Vision;
import frc.robot.vision.limelight.Limelight;
import frc.robot.vision.limelight.LimelightState;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;

public class Robot extends Base581Robot {
  private final Hardware hardware = new Hardware();

  private final Trailblazer trailblazer =
      new Trailblazer(
          new HeuristicPathTracker(new PoseErrorTolerance(0.5, 10)),
          new PidPathFollower(new PIDController(3.5, 0, 0), new PIDController(4.0, 0, 0)));

  private final Limelight turretLimelight =
      new Limelight("turret", LimelightState.TAGS, CameraConfigs.TURRET);
  private final Limelight backLimelight =
      new Limelight("backl", LimelightState.TAGS, CameraConfigs.BACK);
  private final Limelight groundLimelight =
      new Limelight("ground", LimelightState.CLUSTER_MAP, CameraConfigs.GROUND);
  private final HealthManager health =
      new HealthManager(turretLimelight, backLimelight, groundLimelight);
  private final Swerve swerve =
      new Swerve(hardware.drivetrain, health, hardware.driverController, trailblazer);
  private final Imu imu = new Imu(swerve.drivetrain);

  private final ShooterHood shooterHood = new ShooterHood(hardware.shooterHoodMotor);

  private final Shooter shooter =
      new Shooter(hardware.shooterLeftMotor, hardware.shooterRightMotor);
  private final GenericIntake intake =
      RobotKind.IS_COMP_BOT
          ? new IntakeTwoMotor(hardware.intakeLeftMotor, hardware.intakeRightMotor)
          : new Intake(hardware.intakeLeftMotor);
  private final Deploy deploy =
      new Deploy(hardware.deployDifferentialMechanism, hardware.hopperCANRange);
  private final DyeRotor dyeRotor =
      new DyeRotor(hardware.rotorMotor, hardware.horizontalMotor, hardware.verticalMotor);
  private final Vision vision = new Vision(imu, turretLimelight, backLimelight, groundLimelight);
  private final Localization localization =
      new Localization(swerve, hardware.drivetrain, vision, imu);
  private final Turret turret = new Turret(hardware.turretMotor, hardware.turretEncoder, vision);
  private final Climber climber = new Climber(hardware.climbMotor);

  private final ClusterMap clusterMap = new ClusterMap(localization, swerve, groundLimelight);
  private final HubActivity hubActivity = new HubActivity();

  private final RobotManager robotManager =
      new RobotManager(
          shooterHood,
          localization,
          swerve,
          shooter,
          dyeRotor,
          turret,
          intake,
          deploy,
          vision,
          hardware.driverController,
          health,
          hubActivity,
          trailblazer,
          climber,
          clusterMap,
          hardware);

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

    finalizeInit();

    FieldUtil.debugLogFieldZones();

    if (RobotBase.isSimulation()) {
      try {
        var docsDir = Path.of(System.getProperty("user.dir")).resolve("../docs");
        Files.writeString(
            docsDir.resolve("field_obstacles.svg"), FieldUtil.FIELD_OBSTACLES.toSvg());
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

    driver
        .start()
        .onPress(robotManager::startTeleopAutoClimbSequence)
        .onRelease(robotManager::stopTeleopAutoClimbAlignment);

    driver.back().onPress(localization::zeroGyro);

    driver
        .leftTrigger()
        .onPress(() -> robotManager.setDriverWantsIntake(true))
        .onRelease(() -> robotManager.setDriverWantsIntake(false));

    driver.rightTrigger().onPress(robotManager::prepareScoreOrFeedRequest);

    driver.rightBumper().onPress(robotManager::idleRequest);

    operator.start().onPress(robotManager::homeDeployRequest);

    operator.back().onPress(robotManager::homeShooterHoodRequest);

    operator.x().onPress(robotManager::unjamRequest).onRelease(robotManager::idleRequest);

    operator.y().onPress(robotManager::manualClimbSequenceForward);

    operator.b().onPress(robotManager::prepareFeedRequest).onRelease(robotManager::idleRequest);

    operator
        .rightTrigger()
        .onPress(robotManager::prepareScoreRequest)
        .onRelease(robotManager::idleRequest);

    // Use as idle button when not climbing, otherwise does sequence and eventually gets back to
    // idle
    operator.a().onPress(robotManager::manualClimbSequenceBackwardOrIdleRequest);

    operator
        .leftTrigger()
        .onPress(robotManager::stowDeployRequest)
        .onRelease(deploy::intakeRequest);

    operator
        .leftBumper()
        .onPress(robotManager::setFeedGoalLeftRequest)
        .onRelease(robotManager::setFeedGoalClosestRequest);

    operator
        .rightBumper()
        .onPress(robotManager::setFeedGoalRightRequest)
        .onRelease(robotManager::setFeedGoalClosestRequest);
  }
}
