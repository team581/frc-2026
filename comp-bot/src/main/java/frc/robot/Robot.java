package frc.robot;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radian;

import com.team581.Base581Robot;
import com.team581.GlobalConfig;
import com.team581.controller.ControllerBindings;
import com.team581.math.MathHelpers;
import com.team581.math.PoseErrorTolerance;
import com.team581.simkit.FuelSim;
import com.team581.trailblazer.Trailblazer;
import com.team581.trailblazer.followers.PidPathFollower;
import com.team581.trailblazer.trackers.HeuristicPathTracker;
import com.team581.util.FieldUtil;
import com.team581.util.FmsUtil;
import dev.doglog.DogLog;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.autos.Autos;
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

  private final Trailblazer trailblazer =
      new Trailblazer(
          new HeuristicPathTracker(new PoseErrorTolerance(0.5, 10)),
          new PidPathFollower(
              new PIDController(3.5, 0, 0),
              new PIDController(
                  Swerve.ORIGINAL_HEADING_PID.getP(),
                  Swerve.ORIGINAL_HEADING_PID.getI(),
                  Swerve.ORIGINAL_HEADING_PID.getD())));

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
  private final Swerve swerve =
      new Swerve(hardware.drivetrain, health, hardware.driverController, trailblazer);
  private final Imu imu = new Imu(swerve.drivetrain);

  public FuelSim fuelSim = new FuelSim("FuelSim");

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

    finalizeInit();

    if (GlobalConfig.IS_DEVELOPMENT) {
      FieldUtil.debugLogFieldZones();
    }

    if (RobotBase.isSimulation()) {
      try {
        var docsDir = Path.of(System.getProperty("user.dir")).resolve("../docs");
        Files.writeString(
            docsDir.resolve("feeding_obstructions.svg"), FieldUtil.FEEDING_OBSTRUCTIONS.toSvg());
        Files.writeString(
            docsDir.resolve("hub_scoring_obstructions.svg"),
            FieldUtil.HUB_SCORING_OBSTRUCTIONS.toSvg());
      } catch (IOException e) {
        throw new RuntimeException("Failed to write field obstacles SVG", e);
      }
    }

    if (RobotBase.isSimulation()) {
      initFuelSim();
    }

    FieldUtil.debugLogFieldZones();
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
  public void simulationPeriodic() {
    if (hopperManager.isShooting()) {
      fuelSim.launchFuel(
          LinearVelocity.ofBaseUnits(
              MathHelpers.rpmToLinearVelocity(shooter.getAverageRPM(), Units.inchesToMeters(1.6)),
              MetersPerSecond),
          Angle.ofBaseUnits(Units.degreesToRadians(90 - shooterHood.getAngle()), Radian),
          Angle.ofBaseUnits(Math.PI, Radian),
          Distance.ofBaseUnits(
              Units.inchesToMeters(20),
              Meter)); // Spawns a fuel onto the field at the robot's position with a specified
      // launch velocity and angles, accounting for robot movement (robot must be
      // registered)
    }
    fuelSim.updateSim();
  }

  private void initFuelSim() {
    fuelSim.setMaxAdditions(100);

    // Register a robot for collision with fuel
    fuelSim.registerRobot(
        Units.inchesToMeters(34.5), // from left to right in meters
        Units.inchesToMeters(33), // from front to back in meters
        Units.inchesToMeters(4.5), // from floor to top of bumpers in meters
        Units.inchesToMeters(21),
        Units.inchesToMeters(-12),
        Units.inchesToMeters(24.5),
        () -> localization.getPose(), // Supplier<Pose2d> of robot pose
        () -> swerve.getFieldRelativeSpeeds()); // Supplier<ChassisSpeeds> of field-centric chassis
    // speeds

    // Register an intake to remove fuel from the field as a rectangular bounding box
    fuelSim.registerIntake(
        Units.inchesToMeters(16.5),
        Units.inchesToMeters(25),
        -Units.inchesToMeters(17.25),
        Units.inchesToMeters(17.25), // robot-centric coordinates for bounding box in meters
        () -> {
          return hopperManager.isIntaking();
        }); // (optional) Runnable called whenever a fuel is intaked

    fuelSim.setSubticks(
        5); // sets the number of physics iterations to perform per 20ms loop. Default = 5

    fuelSim.start(); // enables the simulation to run (updateSim must still be called periodically)

    fuelSim.enableAirResistance(); // an additional drag force will be applied to fuel in physics
    // update step
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

    driver.rightTrigger().onRelease(robotManager::idleRequest);

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

    operator.b().onPress(robotManager::prepareFeedRequest).onRelease(robotManager::idleRequest);

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
        .onPress(robotManager::warmupScoreOrFeedRequest)
        .onRelease(robotManager::cancelWarmupRequest);
    operator
        .rightBumper()
        .onPress(() -> robotManager.setTrenchOverrideRequest(true))
        .onRelease(() -> robotManager.setTrenchOverrideRequest(false));
  }
}
