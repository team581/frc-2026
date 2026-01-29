package frc.robot;

import com.team581.Base581Robot;
import com.team581.config.CameraConfig;
import com.team581.config.LimelightModel;
import com.team581.math.PoseErrorTolerance;
import com.team581.swerve.DriveSource;
import com.team581.swerve.XboxControllerDriveSource;
import com.team581.trailblazer.Trailblazer;
import com.team581.trailblazer.followers.PidPathFollower;
import com.team581.trailblazer.trackers.HeuristicPathTracker;
import com.team581.util.FieldUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import frc.robot.autos.Autos;
import frc.robot.deploy.Deploy;
import frc.robot.dye_rotor.DyeRotor;
import frc.robot.generated.BuildConstants;
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

  private final DriveSource teleopDriverSource =
      new XboxControllerDriveSource(
          hardware.driverController, Swerve.MAX_SPEED, Swerve.TELEOP_MAX_ANGULAR_RATE);

  private final Swerve swerve = new Swerve(hardware.drivetrain, teleopDriverSource);
  private final Imu imu = new Imu(swerve.drivetrain);

  private final ShooterHood shooterHood = new ShooterHood(hardware.shooterHoodMotor);

  private final Shooter shooter =
      new Shooter(hardware.shooterRightMotor, hardware.shooterLeftMotor);
  private final Intake intake = new Intake(hardware.intakeMotor);
  private final Deploy deploy = new Deploy(hardware.deployMotor);
  private final DyeRotor dyeRotor =
      new DyeRotor(hardware.rotorMotor, hardware.horizontalMotor, hardware.verticalMotor);
  private final Lights lights = new Lights(hardware.candle);
  private final Limelight turretLimelight =
      new Limelight(
          "turret",
          LimelightState.TAGS,
          new CameraConfig(
              LimelightModel.FOUR,
              false,
              Units.inchesToMeters(0.0),
              Units.inchesToMeters(0.0),
              Units.inchesToMeters(0.0),
              0.0,
              0.0,
              0.0));
  private final Limelight backLimelight =
      new Limelight(
          "back",
          LimelightState.TAGS,
          new CameraConfig(
              LimelightModel.THREEG,
              true,
              Units.inchesToMeters(0.0),
              Units.inchesToMeters(0.0),
              Units.inchesToMeters(0.0),
              0.0,
              0.0,
              0.0));
  private final Vision vision = new Vision(imu, turretLimelight, backLimelight);
  private final Localization localization =
      new Localization(swerve, hardware.drivetrain, vision, imu);
  private final Turret turret = new Turret(hardware.turretMotor, vision);

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
          lights);

  @SuppressWarnings("unused") // Registers itself as a subsystem
  private final Autos autos = new Autos(robotManager, trailblazer, teleopDriverSource);

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
    var driverBack = enabledEvent.and(hardware.driverController.back(buttonBindingsLoop));
    driverBack.rising().ifHigh(localization::zeroGyro);

    var driverY = enabledEvent.and(hardware.driverController.y(buttonBindingsLoop));
    driverY.rising().ifHigh(shooter::scoreRequest);
  }
}
