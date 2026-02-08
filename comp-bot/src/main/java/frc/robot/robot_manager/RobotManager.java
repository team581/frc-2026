package frc.robot.robot_manager;

import com.team581.math.MathHelpers;
import com.team581.swerve.SwerveAssist;
import com.team581.trailblazer.Trailblazer;
import com.team581.util.FeedLocation;
import com.team581.util.FieldUtil;
import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.climber.ClimbLocation;
import frc.robot.climber.Climber;
import frc.robot.deploy.Deploy;
import frc.robot.dye_rotor.DyeRotor;
import frc.robot.health.HealthManager;
import frc.robot.intake.Intake;
import frc.robot.intake.IntakeState;
import frc.robot.lights.Lights;
import frc.robot.lights.LightsState;
import frc.robot.localization.Localization;
import frc.robot.shooter.Shooter;
import frc.robot.shooter_hood.ShooterHood;
import frc.robot.swerve.Swerve;
import frc.robot.turret.Turret;
import frc.robot.util.AimParameterUtil;
import frc.robot.util.AimParameterUtil.AimingParameters;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.vision.Vision;
import frc.robot.vision.VisionState;

public class RobotManager extends StateMachineSubsystem<RobotState> {
  public final Localization localization;
  public final Swerve swerve;
  private final ShooterHood shooterHood;
  private final Shooter shooter;
  private final DyeRotor dyeRotor;
  private final Deploy deploy;
  private final Turret turret;
  private final Intake intake;
  private final Vision vision;
  private final Lights lights;
  public final XboxController driverController;
  private final HealthManager health;
  private final Trailblazer trailblazer;
  private final Climber climber;

  private Pose2d robotPose = Pose2d.kZero;
  private boolean nearTrench = false;

  private AimingParameters scoringParameters = new AimingParameters(0, 0);
  private AimingParameters feedingParameters = new AimingParameters(0, 0);
  private static final double PRESET_FEED_DISTANCE = 0.0;
  private boolean isMoving = false;

  private FeedLocation feedLocation = FeedLocation.CLOSEST;

  public RobotManager(
      ShooterHood shooterHood,
      Localization localization,
      Swerve swerve,
      Shooter shooter,
      DyeRotor dyeRotor,
      Turret turret,
      Intake intake,
      Deploy deploy,
      Vision vision,
      Lights lights,
      XboxController driverController,
      HealthManager health,
      Trailblazer trailblazer,
      Climber climber) {
    super(SubsystemPriority.ROBOT_MANAGER, RobotState.IDLE);
    this.shooterHood = shooterHood;
    this.localization = localization;
    this.swerve = swerve;
    this.shooter = shooter;
    this.dyeRotor = dyeRotor;
    this.turret = turret;
    this.intake = intake;
    this.deploy = deploy;
    this.vision = vision;
    this.lights = lights;
    this.driverController = driverController;
    this.health = health;
    this.trailblazer = trailblazer;
    this.climber = climber;
  }

  @Override
  protected RobotState getNextState(RobotState currentState) {
    return switch (currentState) {
      // No auto transitions for these states
      case IDLE,
          UNJAM,
          MANUAL_CLIMB_1_LINEUP_L1,
          MANUAL_CLIMB_2_RAISING_L1,
          MANUAL_CLIMB_3_HANGING_L1,
          MANUAL_CLIMB_4_RAISING_L2,
          MANUAL_CLIMB_5_HANGING_L2,
          MANUAL_CLIMB_6_RAISING_L3,
          MANUAL_CLIMB_7_HANGING_L3,
          AUTOMATIC_CLIMB_7_HANGING_L3 ->
          currentState;
      case PREPARE_SCORE -> {
        if (shooter.atGoal()
            && localization.isTrustworthy()
            && FieldUtil.isRobotInAllianceZone(robotPose.getTranslation())
            && dyeRotor.atGoal()
            && turret.atGoal()
            && shooterHood.atGoal()) {
          yield RobotState.SCORE;
        }
        yield currentState;
      }
      case PREPARE_PRESET_SCORE -> {
        if (shooter.atGoal()
            && dyeRotor.atGoal()
            && turret.atGoal()
            && shooterHood.atGoal()
            && !isMoving) {
          yield RobotState.PRESET_SCORE;
        }
        yield currentState;
      }
      case PREPARE_PRESET_FEED ->
          shooter.atGoal() && dyeRotor.atGoal() && turret.atGoal() && shooterHood.atGoal()
              ? RobotState.PRESET_FEED
              : currentState;

      case PREPARE_FEED ->
          shooter.atGoal()
                  // If localization is healthy, you can feed if we're not in a no-feed zone
                  // If localization is dead, you can always shoot
                  && (health.isLocalizationHealthy()
                      ? !FieldUtil.isRobotInNoFeedZone(robotPose)
                      : true)
                  && dyeRotor.atGoal()
                  && turret.atGoal()
                  && shooterHood.atGoal()
              ? RobotState.FEED
              : currentState;
      case SCORE -> {
        // If we are not in the alliance zone while vision is online, stop tracking the hub.
        // Otherwise, if vision is dead and we cannot reliable track whether we are in the alliance
        // zone, we still want to be able to score
        if (health.isLocalizationHealthy()
            && !FieldUtil.isRobotInAllianceZone(robotPose.getTranslation())) {
          yield RobotState.IDLE;
        }

        if (shooter.atGoal()
            && localization.isTrustworthy()
            && dyeRotor.atGoal()
            && turret.atGoal()
            && shooterHood.atGoal()) {
          yield currentState;
        }

        yield RobotState.PREPARE_SCORE;
      }
      case PRESET_SCORE -> {
        if (shooter.atGoal()
            && dyeRotor.atGoal()
            && turret.atGoal()
            && shooterHood.atGoal()
            && !isMoving) {
          yield currentState;
        }
        yield RobotState.PREPARE_PRESET_SCORE;
      }
      case PRESET_FEED ->
          shooter.atGoal() && dyeRotor.atGoal() && turret.atGoal() && shooterHood.atGoal()
              ? currentState
              : RobotState.PREPARE_PRESET_FEED;
      case FEED ->
          shooter.atGoal()
                  && (health.isLocalizationHealthy()
                      ? !FieldUtil.isRobotInNoFeedZone(robotPose)
                      : true)
                  && dyeRotor.atGoal()
                  && turret.atGoal()
                  && shooterHood.atGoal()
              ? currentState
              : RobotState.PREPARE_FEED;
      case AUTOMATIC_CLIMB_1_LINEUP_L1 -> {
        if (climber.atGoal() && trailblazer.atGoal(robotPose)) {
          yield RobotState.AUTOMATIC_CLIMB_2_RAISING_L1;
        }
        yield currentState;
      }
      case AUTOMATIC_CLIMB_2_RAISING_L1 -> {
        if (climber.atGoal()) {
          yield RobotState.AUTOMATIC_CLIMB_3_HANGING_L1;
        }
        yield currentState;
      }
      case AUTOMATIC_CLIMB_3_HANGING_L1 -> {
        if (climber.atGoal()) {
          yield RobotState.AUTOMATIC_CLIMB_4_RAISING_L2;
        }
        yield currentState;
      }
      case AUTOMATIC_CLIMB_4_RAISING_L2 -> {
        if (climber.atGoal()) {
          yield RobotState.AUTOMATIC_CLIMB_5_HANGING_L2;
        }
        yield currentState;
      }
      case AUTOMATIC_CLIMB_5_HANGING_L2 -> {
        if (climber.atGoal()) {
          yield RobotState.AUTOMATIC_CLIMB_6_RAISING_L3;
        }
        yield currentState;
      }
      case AUTOMATIC_CLIMB_6_RAISING_L3 -> {
        if (climber.atGoal()) {
          yield RobotState.AUTOMATIC_CLIMB_7_HANGING_L3;
        }
        yield currentState;
      }
      case CLIMB_1_LINEUP_L1_AUTONOMOUS -> {
        if (climber.atGoal()) {
          yield RobotState.CLIMB_2_RAISING_L1_AUTONOMOUS;
        }
        yield currentState;
      }
      case CLIMB_2_RAISING_L1_AUTONOMOUS -> {
        if (climber.atGoal()) {
          yield RobotState.CLIMB_3_HANGING_L1_AUTONOMOUS;
        }
        yield currentState;
      }
      case CLIMB_3_HANGING_L1_AUTONOMOUS -> {
        if (climber.atGoal() && DriverStation.isTeleop()) {
          yield RobotState.CLIMB_4_RELEASE_L1_AUTONOMOUS;
        }
        yield currentState;
      }
      case CLIMB_4_RELEASE_L1_AUTONOMOUS -> {
        if (climber.atGoal()) {
          yield RobotState.IDLE;
        }
        yield currentState;
      }
    };
  }

  @Override
  protected void afterTransition(RobotState newState) {
    switch (newState) {
      case IDLE -> {
        vision.setState(VisionState.TAGS);
        shooter.idleRequest();
        // Set hood behavior separately while idling
        dyeRotor.idleRequest();
        // Set turret behavior separately while idling
        deploy.intakeRequest();
        intake.idleRequest();
        swerve.normalDriveRequest();
        lights.setState(LightsState.IDLE_INTAKE_NOT_FULL);
        climber.stowRequest();
      }
      case PREPARE_FEED -> {
        vision.setState(VisionState.TAGS);
        shooter.feedRequest(feedingParameters.distance());
        shooterHood.feedRequest(feedingParameters.distance());
        dyeRotor.shootRequest();
        turret.feedRequest(feedingParameters.turretAngle());
        // Deploy is controlled separately
        // Intake is controlled separately
        swerve.normalDriveRequest();
        lights.setState(LightsState.WAITING_TO_SHOOT);
        climber.stowRequest();
      }
      case FEED -> {
        vision.setState(VisionState.TAGS);
        shooter.feedRequest(feedingParameters.distance());
        shooterHood.feedRequest(feedingParameters.distance());
        dyeRotor.shootRequest();
        turret.feedRequest(feedingParameters.turretAngle());
        deploy.shootingRequest();
        intake.shootingRequest();
        swerve.normalDriveRequest();
        lights.setState(LightsState.SHOOTING);
        climber.stowRequest();
      }
      case PREPARE_SCORE -> {
        vision.setState(VisionState.HUB_TAGS);
        shooter.scoreRequest(scoringParameters.distance());
        shooterHood.scoreRequest(scoringParameters.distance());
        dyeRotor.shootRequest();
        // Turret is controlled depending on what zone we're in
        // Deploy is controlled separately
        // Intake is controlled separately
        swerve.normalDriveRequest();
        lights.setState(LightsState.WAITING_TO_SHOOT);
        climber.stowRequest();
      }
      case SCORE -> {
        vision.setState(VisionState.HUB_TAGS);
        shooter.scoreRequest(scoringParameters.distance());
        shooterHood.scoreRequest(scoringParameters.distance());
        dyeRotor.shootRequest();
        turret.scoreRequest(scoringParameters.turretAngle());
        deploy.shootingRequest();
        intake.shootingRequest();
        swerve.normalDriveRequest();
        lights.setState(LightsState.SHOOTING);
        climber.stowRequest();
      }
      case PREPARE_PRESET_FEED -> {
        // Vision is busted
        shooter.feedRequest(PRESET_FEED_DISTANCE);
        shooterHood.feedRequest(PRESET_FEED_DISTANCE);
        dyeRotor.shootRequest();
        turret.feedRequest(0);
        // Deploy is controlled separately
        // Intake is controlled separately
        swerve.normalDriveRequest();
        lights.setState(LightsState.WAITING_TO_SHOOT);
        climber.stowRequest();
      }
      case PRESET_FEED -> {
        // Vision is busted
        shooter.feedRequest(PRESET_FEED_DISTANCE);
        shooterHood.feedRequest(PRESET_FEED_DISTANCE);
        dyeRotor.shootRequest();
        turret.feedRequest(0);
        deploy.shootingRequest();
        intake.shootingRequest();
        swerve.normalDriveRequest();
        lights.setState(LightsState.SHOOTING);
        climber.stowRequest();
      }
      case PREPARE_PRESET_SCORE -> {
        // Vision is busted
        shooter.scoreRequest(scoringParameters.distance());
        shooterHood.scoreRequest(scoringParameters.distance());
        dyeRotor.shootRequest();
        turret.scoreRequest(scoringParameters.turretAngle());
        // Deploy is controlled separately
        // Intake is controlled separately
        swerve.normalDriveRequest();
        lights.setState(LightsState.WAITING_TO_SHOOT);
        climber.stowRequest();
      }
      case PRESET_SCORE -> {
        // Vision is busted
        shooter.scoreRequest(scoringParameters.distance());
        shooterHood.scoreRequest(scoringParameters.distance());
        dyeRotor.shootRequest();
        turret.scoreRequest(scoringParameters.turretAngle());
        deploy.shootingRequest();
        intake.shootingRequest();
        swerve.normalDriveRequest();
        lights.setState(LightsState.SHOOTING);
        climber.stowRequest();
      }
      case UNJAM -> {
        vision.setState(VisionState.TAGS);
        shooter.idleRequest();
        shooterHood.idleRequest();
        dyeRotor.unjamRequest();
        // Set turret behavior separately
        // Deploy is controlled separately
        intake.idleRequest();
        swerve.normalDriveRequest();
        lights.setState(LightsState.IDLE_INTAKE_NOT_FULL);
        climber.stowRequest();
      }
      case MANUAL_CLIMB_1_LINEUP_L1 -> {
        vision.setState(VisionState.TAGS);
        shooter.idleRequest();
        shooterHood.idleRequest();
        dyeRotor.idleRequest();
        // Set turret behavior seperate while climbing
        deploy.stowRequest();
        intake.idleRequest();
        swerve.normalDriveRequest();
        lights.setState(LightsState.CLIMB_1);
        climber.l1LineupRequest();
      }
      case MANUAL_CLIMB_2_RAISING_L1 -> {
        vision.setState(VisionState.TAGS);
        shooter.idleRequest();
        shooterHood.idleRequest();
        dyeRotor.idleRequest();
        // Set turret behavior separate climbing
        deploy.stowRequest();
        intake.idleRequest();
        swerve.normalDriveRequest();
        lights.setState(LightsState.CLIMB_2);
        climber.l1LineupRequest();
      }
      case MANUAL_CLIMB_3_HANGING_L1 -> {
        vision.setState(VisionState.TAGS);
        shooter.idleRequest();
        shooterHood.idleRequest();
        dyeRotor.idleRequest();
        // Set turret behavior separate climbing
        deploy.stowRequest();
        intake.idleRequest();
        swerve.normalDriveRequest();
        lights.setState(LightsState.CLIMB_3);
        climber.l1HangingRequest();
      }
      case MANUAL_CLIMB_4_RAISING_L2 -> {
        vision.setState(VisionState.TAGS);
        shooter.idleRequest();
        shooterHood.idleRequest();
        dyeRotor.idleRequest();
        // Set turret behavior separate climbing
        deploy.stowRequest();
        intake.idleRequest();
        swerve.normalDriveRequest();
        lights.setState(LightsState.CLIMB_4);
        climber.l2LineupRequest();
      }
      case MANUAL_CLIMB_5_HANGING_L2 -> {
        vision.setState(VisionState.TAGS);
        shooter.idleRequest();
        shooterHood.idleRequest();
        dyeRotor.idleRequest();
        // Set turret behavior separate climbing
        deploy.stowRequest();
        intake.idleRequest();
        swerve.normalDriveRequest();
        lights.setState(LightsState.CLIMB_5);
        climber.l2HangingRequest();
      }
      case MANUAL_CLIMB_6_RAISING_L3 -> {
        vision.setState(VisionState.TAGS);
        shooter.idleRequest();
        shooterHood.idleRequest();
        dyeRotor.idleRequest();
        // Set turret behavior separate climbing
        deploy.stowRequest();
        intake.idleRequest();
        swerve.normalDriveRequest();
        lights.setState(LightsState.CLIMB_6);
        climber.l3LineupRequest();
      }
      case MANUAL_CLIMB_7_HANGING_L3 -> {
        vision.setState(VisionState.TAGS);
        shooter.idleRequest();
        shooterHood.idleRequest();
        dyeRotor.idleRequest();
        // Set turret behavior separate climbing
        deploy.stowRequest();
        intake.idleRequest();
        swerve.normalDriveRequest();
        lights.setState(LightsState.CLIMB_7);
        climber.l3HangingRequest();
      }
      case AUTOMATIC_CLIMB_1_LINEUP_L1 -> {
        vision.setState(VisionState.TAGS);
        shooter.idleRequest();
        shooterHood.idleRequest();
        dyeRotor.idleRequest();
        // Set turret behavior separate climbing
        deploy.stowRequest();
        intake.idleRequest();
        trailblazer.setActiveSegment(
            ClimbAssist.getClimbAssistSegment(robotPose, ClimbLocation.CLOSEST));
        swerve.climbAssistDriveRequest();
        lights.setState(LightsState.CLIMB_1);
        climber.l1LineupRequest();
      }
      case AUTOMATIC_CLIMB_2_RAISING_L1 -> {
        vision.setState(VisionState.TAGS);
        shooter.idleRequest();
        shooterHood.idleRequest();
        dyeRotor.idleRequest();
        // Set turret behavior separate climbing
        deploy.stowRequest();
        intake.idleRequest();
        swerve.normalDriveRequest();
        lights.setState(LightsState.CLIMB_2);
        climber.l1LineupRequest();
      }
      case AUTOMATIC_CLIMB_3_HANGING_L1 -> {
        vision.setState(VisionState.TAGS);
        shooter.idleRequest();
        shooterHood.idleRequest();
        dyeRotor.idleRequest();
        // Set turret behavior separate climbing
        deploy.stowRequest();
        intake.idleRequest();
        swerve.normalDriveRequest();
        lights.setState(LightsState.CLIMB_3);
        climber.l1HangingRequest();
      }
      case AUTOMATIC_CLIMB_4_RAISING_L2 -> {
        vision.setState(VisionState.TAGS);
        shooter.idleRequest();
        shooterHood.idleRequest();
        dyeRotor.idleRequest();
        // Set turret behavior separate climbing
        deploy.stowRequest();
        intake.idleRequest();
        swerve.normalDriveRequest();
        lights.setState(LightsState.CLIMB_4);
        climber.l2LineupRequest();
      }
      case AUTOMATIC_CLIMB_5_HANGING_L2 -> {
        vision.setState(VisionState.TAGS);
        shooter.idleRequest();
        shooterHood.idleRequest();
        dyeRotor.idleRequest();
        // Set turret behavior separate climbing
        deploy.stowRequest();
        intake.idleRequest();
        swerve.normalDriveRequest();
        lights.setState(LightsState.CLIMB_5);
        climber.l2HangingRequest();
      }
      case AUTOMATIC_CLIMB_6_RAISING_L3 -> {
        vision.setState(VisionState.TAGS);
        shooter.idleRequest();
        shooterHood.idleRequest();
        dyeRotor.idleRequest();
        // Set turret behavior separate climbing
        deploy.stowRequest();
        intake.idleRequest();
        swerve.normalDriveRequest();
        lights.setState(LightsState.CLIMB_6);
        climber.l3LineupRequest();
      }
      case AUTOMATIC_CLIMB_7_HANGING_L3 -> {
        vision.setState(VisionState.TAGS);
        shooter.idleRequest();
        shooterHood.idleRequest();
        dyeRotor.idleRequest();
        // Set turret behavior separate climbing
        deploy.stowRequest();
        intake.idleRequest();
        swerve.normalDriveRequest();
        lights.setState(LightsState.CLIMB_7);
        climber.l3HangingRequest();
      }
      case CLIMB_1_LINEUP_L1_AUTONOMOUS -> {
        vision.setState(VisionState.TAGS);
        shooter.idleRequest();
        shooterHood.idleRequest();
        dyeRotor.idleRequest();
        // Set turret behavior separate climbing
        deploy.stowRequest();
        intake.idleRequest();
        swerve.normalDriveRequest();
        lights.setState(LightsState.CLIMB_1);
        climber.l1LineupRequest();
      }
      case CLIMB_2_RAISING_L1_AUTONOMOUS -> {
        vision.setState(VisionState.TAGS);
        shooter.idleRequest();
        shooterHood.idleRequest();
        dyeRotor.idleRequest();
        // Set turret behavior separate climbing
        deploy.stowRequest();
        intake.idleRequest();
        swerve.normalDriveRequest();
        lights.setState(LightsState.CLIMB_2);
        climber.l1LineupRequest();
      }
      case CLIMB_3_HANGING_L1_AUTONOMOUS -> {
        vision.setState(VisionState.TAGS);
        shooter.idleRequest();
        shooterHood.idleRequest();
        dyeRotor.idleRequest();
        // Set turret behavior separate climbing
        deploy.stowRequest();
        intake.idleRequest();
        swerve.normalDriveRequest();
        lights.setState(LightsState.CLIMB_3);
        climber.l1HangingRequest();
      }
      case CLIMB_4_RELEASE_L1_AUTONOMOUS -> {
        vision.setState(VisionState.TAGS);
        shooter.idleRequest();
        shooterHood.idleRequest();
        dyeRotor.idleRequest();
        // Set turret behavior separate climbing
        deploy.stowRequest();
        intake.idleRequest();
        swerve.normalDriveRequest();
        lights.setState(LightsState.CLIMB_4);
        // TODO: check this logic
        climber.l1LineupRequest();
      }
    }
  }

  @Override
  protected void whileInState(RobotState state) {
    switch (state) {
      case IDLE, UNJAM -> {
        smartTurretHoodIdleRequest();
      }
      case PREPARE_SCORE -> {
        smartTurretHoodPrepareScoreRequest();
      }
      case SCORE -> {
        turret.scoreRequest(scoringParameters.turretAngle());
      }
      case PREPARE_FEED -> {
        turret.feedRequest(feedingParameters.turretAngle());
      }
      case FEED -> {
        turret.feedRequest(feedingParameters.turretAngle());
      }
      case PREPARE_PRESET_SCORE -> {
        // Automatically update scoring parameters with preset pose
        if (isMoving) {
          shooterHood.idleRequest();
        } else {
          shooterHood.scoreRequest(scoringParameters.distance());
        }
        turret.scoreRequest(scoringParameters.turretAngle());
      }
      case PRESET_SCORE -> {
        // Automatically update scoring parameters with preset pose
        shooterHood.scoreRequest(scoringParameters.distance());
        turret.scoreRequest(scoringParameters.turretAngle());
      }
      case PREPARE_PRESET_FEED -> {
        // TODO: Get turret feed angle
        turret.feedRequest(0);
      }
      case PRESET_FEED -> {
        // TODO: get turret feed angle
        turret.feedRequest(0);
      }
      case AUTOMATIC_CLIMB_1_LINEUP_L1 -> {
        turret.climbRequest(robotPose);
        swerve.climbAssistDriveRequest();
      }
      case AUTOMATIC_CLIMB_2_RAISING_L1,
          AUTOMATIC_CLIMB_3_HANGING_L1,
          AUTOMATIC_CLIMB_4_RAISING_L2,
          AUTOMATIC_CLIMB_5_HANGING_L2,
          AUTOMATIC_CLIMB_6_RAISING_L3,
          AUTOMATIC_CLIMB_7_HANGING_L3,
          CLIMB_1_LINEUP_L1_AUTONOMOUS,
          CLIMB_2_RAISING_L1_AUTONOMOUS,
          CLIMB_3_HANGING_L1_AUTONOMOUS,
          CLIMB_4_RELEASE_L1_AUTONOMOUS,
          MANUAL_CLIMB_1_LINEUP_L1,
          MANUAL_CLIMB_2_RAISING_L1,
          MANUAL_CLIMB_3_HANGING_L1,
          MANUAL_CLIMB_4_RAISING_L2,
          MANUAL_CLIMB_5_HANGING_L2,
          MANUAL_CLIMB_6_RAISING_L3,
          MANUAL_CLIMB_7_HANGING_L3 -> {
        turret.climbRequest(robotPose);
      }
      default -> {}
    }
    DogLog.log("RobotManager/FeedLocation", feedLocation);
    DogLog.log("RobotManager/FeedParameters", feedingParameters);
    DogLog.log("RobotManager/ScoringParameters", scoringParameters);

    if (!getState().isClimbing()) {
      if (intake.getState() == IntakeState.INTAKING) {
        swerve.intakeDriveRequest();
      } else {
        swerve.normalDriveRequest();
      }
    }

    MechanismVisualizer.log(
        robotPose,
        turret.getAngle(),
        shooterHood.getAngle(),
        deploy.getPosition(),
        0,
        dyeRotor.getAngle());
  }

  private void smartTurretHoodIdleRequest() {
    // -First, if cameras are offline or we are near a trench, always be idle
    // -Otherwise if we are in our alliance zone, point towards hub
    // -And if we are not in alliance zone, point towards feed pose
    if (!health.isLocalizationHealthy() || nearTrench) {
      shooterHood.idleRequest();
      turret.idleScoreRequest(scoringParameters.turretAngle());

      DogLog.log("RobotManager/SmartTurretHoodIdleRequest", "NearTrench");
    } else if (FieldUtil.isRobotPastObstacleTowardAllianceZone(robotPose.getTranslation())) {
      shooterHood.scoreRequest(scoringParameters.distance());
      turret.idleScoreRequest(scoringParameters.turretAngle());

      DogLog.log("RobotManager/SmartTurretHoodIdleRequest", "InAllianceZone");
    } else {
      shooterHood.feedRequest(feedingParameters.distance());
      turret.idleFeedRequest(feedingParameters.turretAngle());

      DogLog.log("RobotManager/SmartTurretHoodIdleRequest", "NotInAlliance");
    }
  }

  private void smartTurretHoodPrepareScoreRequest() {
    // Turret behavior
    if (FieldUtil.isRobotPastObstacleTowardAllianceZone(robotPose.getTranslation())) {
      turret.scoreRequest(scoringParameters.turretAngle());
    } else {
      turret.idleScoreRequest(scoringParameters.turretAngle());
    }

    if (!health.isLocalizationHealthy() || nearTrench) {
      shooterHood.idleRequest();
      DogLog.log("RobotManager/SmartTurretHoodIdleRequest", "NearTrench");
    } else {
      shooterHood.scoreRequest(scoringParameters.distance());
    }
  }

  public void idleRequest() {
    if (!getState().isClimbing()) {
      setStateFromRequest(RobotState.IDLE);
    }
  }

  public void prepareScoreRequest() {
    if (!getState().isClimbing()) {
      if (!health.isLocalizationHealthy()) {
        setStateFromRequest(RobotState.PREPARE_PRESET_SCORE);
      } else {
        setStateFromRequest(RobotState.PREPARE_SCORE);
      }
    }
  }

  public void prepareFeedRequest() {
    if (!getState().isClimbing()) {
      if (!health.isLocalizationHealthy()) {
        setStateFromRequest(RobotState.PREPARE_PRESET_FEED);
      } else {
        setStateFromRequest(RobotState.PREPARE_FEED);
      }
    }
  }

  public void setFeedGoalLeftRequest() {
    feedLocation = FeedLocation.LEFT;
  }

  public void setFeedGoalRightRequest() {
    feedLocation = FeedLocation.RIGHT;
  }

  public void setFeedGoalClosestRequest() {
    feedLocation = FeedLocation.CLOSEST;
  }

  public void intakeRequest() {
    intake.intakeRequest();
    deploy.intakeRequest();
  }

  public void cancelIntakeRequest() {
    intake.idleRequest();
  }

  public void stowDeployRequest() {
    intake.idleRequest();
    deploy.stowRequest();
  }

  public void unjamRequest() {
    if (!getState().isClimbing()) {
      setStateFromRequest(RobotState.UNJAM);
    }
  }

  public void homeDeployRequest() {
    if (!getState().isClimbing()) {
      deploy.homingRequest();
    }
  }

  public void homeShooterHoodRequest() {
    if (!getState().isClimbing()) {
      shooterHood.homingRequest();
    }
  }

  public void startAutoClimbSequence() {
    setStateFromRequest(RobotState.CLIMB_1_LINEUP_L1_AUTONOMOUS);
  }

  public void startTeleopAutoClimbSequence() {
    if (!getState().isClimbing()) {
      setStateFromRequest(RobotState.AUTOMATIC_CLIMB_1_LINEUP_L1);
    }
  }

  public void manualClimbSequenceForward() {
    switch (getState()) {
      default -> setStateFromRequest(RobotState.MANUAL_CLIMB_1_LINEUP_L1);
      case MANUAL_CLIMB_1_LINEUP_L1, AUTOMATIC_CLIMB_1_LINEUP_L1 ->
          setStateFromRequest(RobotState.MANUAL_CLIMB_2_RAISING_L1);
      case MANUAL_CLIMB_2_RAISING_L1, AUTOMATIC_CLIMB_2_RAISING_L1 ->
          setStateFromRequest(RobotState.MANUAL_CLIMB_3_HANGING_L1);
      case MANUAL_CLIMB_3_HANGING_L1, AUTOMATIC_CLIMB_3_HANGING_L1 ->
          setStateFromRequest(RobotState.MANUAL_CLIMB_4_RAISING_L2);

      case MANUAL_CLIMB_4_RAISING_L2, AUTOMATIC_CLIMB_4_RAISING_L2 ->
          setStateFromRequest(RobotState.MANUAL_CLIMB_5_HANGING_L2);
      case MANUAL_CLIMB_5_HANGING_L2, AUTOMATIC_CLIMB_5_HANGING_L2 ->
          setStateFromRequest(RobotState.MANUAL_CLIMB_6_RAISING_L3);

      case MANUAL_CLIMB_6_RAISING_L3, AUTOMATIC_CLIMB_6_RAISING_L3 ->
          setStateFromRequest(RobotState.MANUAL_CLIMB_7_HANGING_L3);
      case MANUAL_CLIMB_7_HANGING_L3, AUTOMATIC_CLIMB_7_HANGING_L3 -> {}
    }
  }

  public void manualClimbSequenceBackward() {
    switch (getState()) {
      default -> {}
      case CLIMB_1_LINEUP_L1_AUTONOMOUS -> setStateFromRequest(RobotState.IDLE);
      case CLIMB_2_RAISING_L1_AUTONOMOUS ->
          setStateFromRequest(RobotState.CLIMB_1_LINEUP_L1_AUTONOMOUS);
      case CLIMB_3_HANGING_L1_AUTONOMOUS ->
          setStateFromRequest(RobotState.CLIMB_2_RAISING_L1_AUTONOMOUS);

      case MANUAL_CLIMB_1_LINEUP_L1, AUTOMATIC_CLIMB_1_LINEUP_L1 ->
          setStateFromRequest(RobotState.IDLE);
      case MANUAL_CLIMB_2_RAISING_L1, AUTOMATIC_CLIMB_2_RAISING_L1 ->
          setStateFromRequest(RobotState.MANUAL_CLIMB_1_LINEUP_L1);
      case MANUAL_CLIMB_3_HANGING_L1, AUTOMATIC_CLIMB_3_HANGING_L1 ->
          setStateFromRequest(RobotState.MANUAL_CLIMB_2_RAISING_L1);

      case MANUAL_CLIMB_4_RAISING_L2, AUTOMATIC_CLIMB_4_RAISING_L2 ->
          setStateFromRequest(RobotState.MANUAL_CLIMB_3_HANGING_L1);
      case MANUAL_CLIMB_5_HANGING_L2, AUTOMATIC_CLIMB_5_HANGING_L2 ->
          setStateFromRequest(RobotState.MANUAL_CLIMB_4_RAISING_L2);

      case MANUAL_CLIMB_6_RAISING_L3, AUTOMATIC_CLIMB_6_RAISING_L3 ->
          setStateFromRequest(RobotState.MANUAL_CLIMB_5_HANGING_L2);
      case MANUAL_CLIMB_7_HANGING_L3, AUTOMATIC_CLIMB_7_HANGING_L3 ->
          setStateFromRequest(RobotState.MANUAL_CLIMB_6_RAISING_L3);
    }
  }

  @Override
  protected void collectInputs() {
    robotPose = localization.getPose();
    vision.setEstimatedPoseAngle(robotPose.getRotation().getDegrees());
    turret.setRobotRotationRate(swerve.getFieldRelativeSpeeds().omegaRadiansPerSecond);
    var speeds = swerve.getFieldRelativeSpeeds();
    isMoving = MathHelpers.getLinearVelocity(speeds) > 0.2;

    nearTrench =
        FieldUtil.inTrench(robotPose.getTranslation())
            || SwerveAssist.ableToTrenchAssist(robotPose, swerve.getFieldRelativeSpeeds());
    var scoringDistance = AimParameterUtil.getScoringDistance(robotPose);
    var feedingDistance = AimParameterUtil.getFeedingDistance(feedLocation, robotPose);

    scoringParameters =
        AimParameterUtil.getScoringParameters(
            // TODO: This should require you to pass in the distance to get the ToF
            health.isLocalizationHealthy()
                ? robotPose
                : new Pose2d(
                    FieldUtil.getFallbackScorePoint().getTranslation(), robotPose.getRotation()),
            swerve.getFieldRelativeSpeeds(),
            shooter.getScoreTimeOfFlight(scoringDistance));
    feedingParameters =
        AimParameterUtil.getFeedingParameters(
            feedLocation,
            robotPose,
            swerve.getFieldRelativeSpeeds(),
            shooter.getFeedTimeOfFlight(feedingDistance));
  }
}
