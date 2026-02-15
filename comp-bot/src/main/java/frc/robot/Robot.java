package frc.robot;

import com.team581.Base581Robot;
import com.team581.config.CameraConfig;
import com.team581.config.LimelightModel;
import com.team581.math.PoseErrorTolerance;
import com.team581.trailblazer.Trailblazer;
import com.team581.trailblazer.followers.PidPathFollower;
import com.team581.trailblazer.trackers.HeuristicPathTracker;
import com.team581.util.FieldUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import frc.robot.autos.Autos;
import frc.robot.climber.Climber;
import frc.robot.deploy.Deploy;
import frc.robot.dye_rotor.DyeRotor;
import frc.robot.generated.BuildConstants;
import frc.robot.health.HealthManager;
import frc.robot.imu.Imu;
import frc.robot.intake.Intake;
import frc.robot.lights.Lights;
import frc.robot.localization.Localization;
import frc.robot.robot_manager.RobotManager;
import frc.robot.shooter.Shooter;
import frc.robot.shooter_hood.ShooterHood;
import frc.robot.swerve.Swerve;
import frc.robot.turret.Turret;
import frc.robot.vision.Vision;
import frc.robot.vision.limelight.Limelight;
import frc.robot.vision.limelight.LimelightState;

public class Robot extends Base581Robot {
  private final Hardware hardware = new Hardware();

  private final Trailblazer trailblazer =
      new Trailblazer(
          new HeuristicPathTracker(new PoseErrorTolerance(0.5, 10)),
          new PidPathFollower(new PIDController(3.5, 0, 0), new PIDController(4.0, 0, 0)));

  private final Limelight turretLimelight =
      new Limelight(
          "turret",
          LimelightState.TAGS,
          new CameraConfig(
              LimelightModel.FOUR,
              true,
              Units.inchesToMeters(0.0),
              Units.inchesToMeters(0.0),
              Units.inchesToMeters(20.432677 + 0.25),
              30.0,
              0.0,
              0.0));
  private final Limelight backLimelight =
      new Limelight(
          "backl",
          LimelightState.TAGS,
          new CameraConfig(
              LimelightModel.FOUR,
              true,
              // back
              Units.inchesToMeters(-13.389),
              // left
              Units.inchesToMeters(-8.3370),
              Units.inchesToMeters(19.7564),
              18.52,
              180.0 + 4.5,
              0.83));

  // ground when stowed
  // Units.inchesToMeters(12.9742),
  // Units.inchesToMeters(0.0),
  // Units.inchesToMeters(16.8886)

  private final Limelight groundLimelight =
      new Limelight(
          "ground",
          LimelightState.CLUSTER_MAP,
          new CameraConfig(
              LimelightModel.THREE,
              true,
              Units.inchesToMeters(25.671),
              Units.inchesToMeters(0.0),
              Units.inchesToMeters(12.9525),
              -20.0,
              0.0,
              0.0));
  private final HealthManager health = new HealthManager(turretLimelight, backLimelight);
  private final Swerve swerve =
      new Swerve(hardware.drivetrain, health, hardware.driverController, trailblazer);
  private final Imu imu = new Imu(swerve.drivetrain);

  private final ShooterHood shooterHood = new ShooterHood(hardware.shooterHoodMotor);

  private final Shooter shooter =
      new Shooter(hardware.shooterLeftMotor, hardware.shooterRightMotor);
  private final Intake intake = new Intake(hardware.intakeMotor);
  private final Deploy deploy =
      new Deploy(hardware.leftDeployMotor, hardware.rightDeployMotor, hardware.hopperCANRange);
  private final DyeRotor dyeRotor =
      new DyeRotor(hardware.rotorMotor, hardware.horizontalMotor, hardware.verticalMotor);
  private final Lights lights = new Lights(hardware.candle);
  private final Vision vision = new Vision(imu, turretLimelight, backLimelight, groundLimelight);
  private final Localization localization =
      new Localization(swerve, hardware.drivetrain, vision, imu);
  private final Turret turret = new Turret(hardware.turretMotor, hardware.turretEncoder, vision);
  private final Climber climber = new Climber(hardware.climbMotor);
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
          lights,
          hardware.driverController,
          health,
          trailblazer,
          climber,
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
  }

  @Override
  protected void configureBindings() {
    var driverStart = enabledEvent.and(hardware.driverController.start(buttonBindingsLoop));
    driverStart.rising().ifHigh(robotManager::startTeleopAutoClimbSequence);

    var driverBack = enabledEvent.and(hardware.driverController.back(buttonBindingsLoop));
    driverBack.rising().ifHigh(localization::zeroGyro);

    var driverLeftTrigger =
        enabledEvent.and(hardware.driverController.leftTrigger(buttonBindingsLoop));
    driverLeftTrigger.rising().ifHigh(robotManager::intakeRequest);
    driverLeftTrigger.falling().ifHigh(robotManager::cancelIntakeRequest);

    var driverRightTrigger =
        enabledEvent.and(hardware.driverController.rightTrigger(buttonBindingsLoop));
    driverRightTrigger.rising().ifHigh(robotManager::prepareScoreRequest);
    driverRightTrigger.falling().ifHigh(robotManager::idleRequest);

    var driverRightBumper =
        enabledEvent.and(hardware.driverController.rightBumper(buttonBindingsLoop));
    driverRightBumper.rising().ifHigh(robotManager::prepareFeedRequest);
    driverRightBumper.falling().ifHigh(robotManager::idleRequest);

    var operatorStart = enabledEvent.and(hardware.operatorController.start(buttonBindingsLoop));
    operatorStart.rising().ifHigh(robotManager::homeDeployRequest);

    var operatorBack = enabledEvent.and(hardware.operatorController.back(buttonBindingsLoop));
    operatorBack.rising().ifHigh(robotManager::homeShooterHoodRequest);

    var operatorX = enabledEvent.and(hardware.operatorController.x(buttonBindingsLoop));
    operatorX.rising().ifHigh(robotManager::unjamRequest);
    operatorX.falling().ifHigh(robotManager::idleRequest);

    var operatorY = enabledEvent.and(hardware.operatorController.y(buttonBindingsLoop));
    operatorY.rising().ifHigh(robotManager::manualClimbSequenceForward);

    var operatorA = enabledEvent.and(hardware.operatorController.a(buttonBindingsLoop));
    operatorA.rising().ifHigh(robotManager::manualClimbSequenceBackward);

    var operatorDpad = enabledEvent.and(hardware.operatorController.pov(90, buttonBindingsLoop));
    operatorDpad.rising().ifHigh(robotManager::idleRequest);

    var operatorLeftTrigger =
        enabledEvent.and(hardware.operatorController.leftTrigger(buttonBindingsLoop));
    operatorLeftTrigger.rising().ifHigh(robotManager::stowDeployRequest);
    operatorLeftTrigger.falling().ifHigh(deploy::intakeRequest);

    var operatorLeftBumper =
        enabledEvent.and(hardware.operatorController.leftBumper(buttonBindingsLoop));
    operatorLeftBumper.rising().ifHigh(robotManager::setFeedGoalLeftRequest);
    operatorLeftBumper.falling().ifHigh(robotManager::setFeedGoalClosestRequest);

    var operatorRightBumper =
        enabledEvent.and(hardware.operatorController.rightBumper(buttonBindingsLoop));
    operatorRightBumper.rising().ifHigh(robotManager::setFeedGoalRightRequest);
    operatorRightBumper.falling().ifHigh(robotManager::setFeedGoalClosestRequest);
  }
}
