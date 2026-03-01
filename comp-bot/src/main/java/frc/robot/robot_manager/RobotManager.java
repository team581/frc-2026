package frc.robot.robot_manager;

import com.team581.math.MathHelpers;
import com.team581.swerve.SwerveAssist;
import com.team581.trailblazer.Trailblazer;
import com.team581.util.FeedLocation;
import com.team581.util.FieldUtil;
import com.team581.util.FmsUtil;
import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Hardware;
import frc.robot.climber.ClimbLocation;
import frc.robot.climber.Climber;
import frc.robot.cluster_map.ClusterMap;
import frc.robot.config.DSOptions;
import frc.robot.config.FeatureFlags;
import frc.robot.deploy.Deploy;
import frc.robot.dye_rotor.DyeRotor;
import frc.robot.health.HealthManager;
import frc.robot.intake.Intake;
import frc.robot.intake.IntakeState;
import frc.robot.localization.Localization;
import frc.robot.shooter.Shooter;
import frc.robot.shooter_hood.ShooterHood;
import frc.robot.swerve.Swerve;
import frc.robot.turret.Turret;
import frc.robot.turret.TurretCalculator;
import frc.robot.util.AimParameterUtil;
import frc.robot.util.AimParameterUtil.AimingParameters;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.vision.Vision;
import frc.robot.vision.VisionState;

public class RobotManager extends StateMachineSubsystem<RobotState> {
  public final Localization localization;
  public final Swerve swerve;
  public final Hardware hardware;
  private final ShooterHood shooterHood;
  private final Shooter shooter;
  private final DyeRotor dyeRotor;
  public final Deploy deploy;
  private final Turret turret;
  private final Intake intake;
  private final Vision vision;
  public final XboxController driverController;
  private final HealthManager health;
  private final Trailblazer trailblazer;
  public final ClusterMap clusterMap;
  private final Climber climber;

  private Pose2d robotPose = Pose2d.kZero;
  private boolean nearTrench = false;

  private boolean climbLocationIsLeft = true;

  private AimingParameters scoringParameters = new AimingParameters(0, 0, 0);
  private AimingParameters feedingParameters = new AimingParameters(0, 0, 0);
  private static final double PRESET_FEED_DISTANCE = 0.0;
  private static final DoubleSubscriber DISTANCE_TO_HUB_THRESHOLD =
      DogLog.tunable("RobotManager/DistanceToHubThreshold", 10.0);
  private boolean isMoving = false;
  private boolean drivingToIntake = false;

  private double timeSinceMatchStart = 0.0;
  private double timeUntilNextShift = 0.0;
  private boolean isHubActive = true;
  private boolean isCloseEnoughToHub = false;
  private final DoubleSubscriber tunableHubStateOffset =
      DogLog.tunable("RobotManager/MatchTimeOffset", 0.0);
  private final Timer teleopTimer = new Timer();

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
      XboxController driverController,
      HealthManager health,
      Trailblazer trailblazer,
      Climber climber,
      ClusterMap clusterMap,
      Hardware hardware) {
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
    this.driverController = driverController;
    this.health = health;
    this.trailblazer = trailblazer;
    this.clusterMap = clusterMap;
    this.climber = climber;

    this.hardware = hardware;
    teleopTimer.start();
  }

  @Override
  public void teleopInit() {
    teleopTimer.reset();
    timeSinceMatchStart = FmsUtil.MATCH_TIME_AT_TELEOP_START;
  }

  @Override
  protected RobotState getNextState(RobotState currentState) {
    return switch (currentState) {
      // No auto transitions for these states
      case UNJAM,
          MANUAL_CLIMB_1_LINEUP_L1,
          MANUAL_CLIMB_2_HANGING_L1,
          MANUAL_CLIMB_3_RAISING_L2,
          MANUAL_CLIMB_4_HANGING_L2,
          MANUAL_CLIMB_5_RAISING_L3,
          MANUAL_CLIMB_6_HANGING_L3,
          FORCE_SCORE,
          AUTOMATIC_CLIMB_7_HANGING_L3,
          CLIMB_8_SCORING_L3 ->
          currentState;
      case STOP_SHOOTING_SCORE,
          STOP_SHOOTING_PRESET_SCORE,
          STOP_SHOOTING_PRESET_FEED,
          STOP_SHOOTING_FEED ->
          timeout(1) ? RobotState.IDLE : currentState;
      case PREPARE_FORCE_SCORE -> {
        if ((FeatureFlags.IGNORE_TURRET_AT_GOAL.getAsBoolean()
                || turret.atGoal(scoringParameters.turretTolerance()))
            && (shooter.atGoal() && !dyeRotor.isJammed() && shooterHood.atGoal())) {
          yield RobotState.FORCE_SCORE;
        }
        yield currentState;
      }
      case IDLE -> {
        if (DSOptions.AUTO_SCORE.getAsBoolean() && isHubActive) {
          yield RobotState.PREPARE_SCORE;
        }
        yield currentState;
      }
      case PREPARE_SCORE -> {
        logScoringTransition();

        if (DSOptions.AUTO_SCORE.getAsBoolean() && !isHubActive) {
          yield RobotState.STOP_SHOOTING_SCORE;
        }
        if ((FeatureFlags.IGNORE_TURRET_AT_GOAL.getAsBoolean()
                || turret.atGoal(scoringParameters.turretTolerance()))
            && (shooter.atGoal()
                && localization.isTrustworthy()
                && FieldUtil.isRobotInAllianceZone(robotPose.getTranslation())
                && !dyeRotor.isJammed()
                && shooterHood.atGoal()
                && isHubActive
                && isCloseEnoughToHub)) {
          yield RobotState.SCORE;
        }
        yield currentState;
      }
      case PREPARE_PRESET_SCORE -> {
        if (shooter.atGoal()
            && !dyeRotor.isJammed()
            && turret.atGoal(scoringParameters.turretTolerance())
            && shooterHood.atGoal()
            && !isMoving) {
          yield RobotState.PRESET_SCORE;
        }
        yield currentState;
      }
      case CLIMB_7_PREPARE_SCORING_L3 -> {
        if (shooter.atGoal() && turret.atGoal() && shooterHood.atGoal() && !dyeRotor.isJammed()) {
          yield RobotState.CLIMB_8_SCORING_L3;
        }
        yield currentState;
      }
      case PREPARE_PRESET_FEED ->
          shooter.atGoal() && !dyeRotor.isJammed() && turret.atGoal() && shooterHood.atGoal()
              ? RobotState.PRESET_FEED
              : currentState;

      case PREPARE_FEED -> {
        logFeedTransition();
        if (shooter.atGoal()
            // If localization is healthy, you can feed if we're not in a no-feed zone
            // If localization is dead, you can always shoot
            && (health.isLocalizationHealthy() ? !FieldUtil.isRobotInNoFeedZone(TurretCalculator.getTurretPose(robotPose)) : true)
            && turret.atGoal(feedingParameters.turretTolerance())
            && shooterHood.atGoal()) {

          yield RobotState.FEED;
        } else {
          yield currentState;
        }
      }
      case SCORE -> {
        logScoringTransition();
        // If we are not in the alliance zone while vision is online, stop tracking the
        // hub.
        // Otherwise, if vision is dead and we cannot reliable track whether we are in
        // the alliance
        // zone, we still want to be able to score
        if (health.isLocalizationHealthy()
            && !FieldUtil.isRobotInAllianceZone(robotPose.getTranslation())) {
          DogLog.timestamp("RobotManager/ScoreTransition/RobotNotInAllianceZone");
          yield RobotState.STOP_SHOOTING_SCORE;
        }

        if (!isHubActive) {
          yield RobotState.STOP_SHOOTING_SCORE;
        }

        if (!FeatureFlags.CANCEL_IN_PROGRESS_SHOT.getAsBoolean()
            || (shooter.atGoal()
                && localization.isTrustworthy()
                && !dyeRotor.isJammed()
                && turret.atGoal(scoringParameters.turretTolerance())
                && shooterHood.atGoal()
                && isCloseEnoughToHub)) {
          yield currentState;
        }

        yield RobotState.PREPARE_SCORE;
      }
      case PRESET_SCORE -> {
        if (shooter.atGoal()
            && !dyeRotor.isJammed()
            && turret.atGoal(scoringParameters.turretTolerance())
            && shooterHood.atGoal()
            && !isMoving) {
          yield currentState;
        }
        yield RobotState.PREPARE_PRESET_SCORE;
      }
      case PRESET_FEED ->
          shooter.atGoal() && !dyeRotor.isJammed() && turret.atGoal() && shooterHood.atGoal()
              ? currentState
              : RobotState.PREPARE_PRESET_FEED;
      case FEED -> {
        logFeedTransition();
        if (!FeatureFlags.CANCEL_IN_PROGRESS_SHOT.getAsBoolean()
            || (shooter.atGoal()
                && (health.isLocalizationHealthy()
                    ? !FieldUtil.isRobotInNoFeedZone(TurretCalculator.getTurretPose(robotPose))
                    : true)
                && turret.atGoal(feedingParameters.turretTolerance())
                && shooterHood.atGoal())) {

          yield currentState;
        } else {

          yield RobotState.PREPARE_FEED;
        }
      }
      case AUTOMATIC_CLIMB_1_APPROACH_L1 -> {
        if (climber.atGoal()) {
          yield RobotState.AUTOMATIC_CLIMB_2_LINEUP_L1;
        }
        yield currentState;
      }
      case AUTOMATIC_CLIMB_2_LINEUP_L1 -> {
        if (climber.atGoal() && trailblazer.atGoal(robotPose)) {
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
        swerve.normalDriveRequest();
        climber.stowRequest();
      }
      case PREPARE_FORCE_SCORE -> {
        vision.setState(VisionState.TAGS);
        shooter.scoreRequest(scoringParameters.distance());
        shooterHood.scoreRequest(scoringParameters.distance());
        dyeRotor.idleRequest();
        turret.scoreRequest(scoringParameters.turretAngle());
        swerve.normalDriveRequest();
        climber.stowRequest();
      }
      case FORCE_SCORE -> {
        vision.setState(VisionState.TAGS);
        shooter.scoreRequest(scoringParameters.distance());
        shooterHood.scoreRequest(scoringParameters.distance());
        dyeRotor.scoreRequest(scoringParameters.distance());
        turret.scoreRequest(scoringParameters.turretAngle());
        deploy.shuffleRequest();
        intake.shootRequest();
        swerve.normalDriveRequest();
        climber.stowRequest();
      }
      case PREPARE_FEED -> {
        vision.setState(VisionState.TAGS);
        shooter.feedRequest(feedingParameters.distance());
        shooterHood.feedRequest(feedingParameters.distance());
        dyeRotor.idleRequest();
        // Deploy is controlled separately
        // Intake is controlled separately
        swerve.normalDriveRequest();
        climber.stowRequest();
      }
      case FEED -> {
        vision.setState(VisionState.TAGS);
        shooter.feedRequest(feedingParameters.distance());
        shooterHood.feedRequest(feedingParameters.distance());
        dyeRotor.feedRequest(feedingParameters.distance());
        turret.feedRequest(feedingParameters.turretAngle());
        deploy.shuffleRequest();
        intake.shootRequest();
        swerve.normalDriveRequest();
        climber.stowRequest();
      }
      case STOP_SHOOTING_FEED -> {
        vision.setState(VisionState.TAGS);
        shooter.feedRequest(feedingParameters.distance());
        shooterHood.feedRequest(feedingParameters.distance());
        dyeRotor.idleRequest();
        turret.feedRequest(feedingParameters.turretAngle());
        deploy.intakeRequest();
        intake.idleRequest();
        swerve.normalDriveRequest();
        climber.stowRequest();
      }
      case PREPARE_SCORE -> {
        vision.setState(VisionState.HUB_TAGS);
        shooter.scoreRequest(scoringParameters.distance());
        dyeRotor.idleRequest();
        // Turret and hood are controlled depending on what zone we're in
        // Deploy is controlled separately
        // Intake is controlled separately
        swerve.rateLimitedDriveRequest();
        deploy.intakeRequest();
        climber.stowRequest();
      }
      case SCORE -> {
        vision.setState(VisionState.HUB_TAGS);
        shooter.scoreRequest(scoringParameters.distance());
        shooterHood.scoreRequest(scoringParameters.distance());
        dyeRotor.scoreRequest(scoringParameters.distance());
        turret.scoreRequest(scoringParameters.turretAngle());
        deploy.shuffleRequest();
        intake.shootRequest();
        swerve.rateLimitedDriveRequest();
        climber.stowRequest();
      }
      case STOP_SHOOTING_SCORE -> {
        vision.setState(VisionState.HUB_TAGS);
        shooter.scoreRequest(scoringParameters.distance());
        shooterHood.scoreRequest(scoringParameters.distance());
        dyeRotor.idleRequest();
        turret.scoreRequest(scoringParameters.turretAngle());
        deploy.intakeRequest();
        intake.idleRequest();
        swerve.rateLimitedDriveRequest();
        climber.stowRequest();
      }
      case PREPARE_PRESET_FEED -> {
        vision.setState(VisionState.TAGS);
        shooter.feedRequest(PRESET_FEED_DISTANCE);
        shooterHood.feedRequest(PRESET_FEED_DISTANCE);
        dyeRotor.idleRequest();
        turret.feedRequest(0);
        // Deploy is controlled separately
        // Intake is controlled separately
        swerve.normalDriveRequest();
        climber.stowRequest();
      }
      case PRESET_FEED -> {
        vision.setState(VisionState.TAGS);
        shooter.feedRequest(PRESET_FEED_DISTANCE);
        shooterHood.feedRequest(PRESET_FEED_DISTANCE);
        dyeRotor.feedRequest(feedingParameters.distance());
        turret.feedRequest(0);
        deploy.shuffleRequest();
        intake.shootRequest();
        swerve.normalDriveRequest();
        climber.stowRequest();
      }
      case STOP_SHOOTING_PRESET_FEED -> {
        vision.setState(VisionState.TAGS);
        shooter.feedRequest(PRESET_FEED_DISTANCE);
        shooterHood.feedRequest(PRESET_FEED_DISTANCE);
        dyeRotor.idleRequest();
        turret.feedRequest(0);
        deploy.intakeRequest();
        intake.idleRequest();
        swerve.normalDriveRequest();
        climber.stowRequest();
      }
      case PREPARE_PRESET_SCORE -> {
        vision.setState(VisionState.TAGS);
        shooter.scoreRequest(scoringParameters.distance());
        shooterHood.scoreRequest(scoringParameters.distance());
        dyeRotor.idleRequest();
        turret.scoreRequest(scoringParameters.turretAngle());
        // Deploy is controlled separately
        // Intake is controlled separately
        swerve.normalDriveRequest();
        climber.stowRequest();
      }
      case PRESET_SCORE -> {
        vision.setState(VisionState.TAGS);
        shooter.scoreRequest(scoringParameters.distance());
        shooterHood.scoreRequest(scoringParameters.distance());
        dyeRotor.scoreRequest(scoringParameters.distance());
        turret.scoreRequest(scoringParameters.turretAngle());
        deploy.shuffleRequest();
        intake.shootRequest();
        swerve.normalDriveRequest();
        climber.stowRequest();
      }
      case STOP_SHOOTING_PRESET_SCORE -> {
        vision.setState(VisionState.TAGS);
        shooter.scoreRequest(scoringParameters.distance());
        shooterHood.scoreRequest(scoringParameters.distance());
        dyeRotor.idleRequest();
        turret.scoreRequest(scoringParameters.turretAngle());
        intake.idleRequest();
        deploy.intakeRequest();
        swerve.normalDriveRequest();
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
        climber.l1LineupRequest();
      }
      case MANUAL_CLIMB_2_HANGING_L1 -> {
        vision.setState(VisionState.TAGS);
        shooter.idleRequest();
        shooterHood.idleRequest();
        dyeRotor.idleRequest();
        // Set turret behavior separate climbing
        deploy.stowRequest();
        intake.idleRequest();
        swerve.normalDriveRequest();
        climber.l1HangingRequest();
      }
      case MANUAL_CLIMB_3_RAISING_L2 -> {
        vision.setState(VisionState.TAGS);
        shooter.idleRequest();
        shooterHood.idleRequest();
        dyeRotor.idleRequest();
        // Set turret behavior separate climbing
        deploy.stowRequest();
        intake.idleRequest();
        swerve.normalDriveRequest();
        climber.l2LineupRequest();
      }
      case MANUAL_CLIMB_4_HANGING_L2 -> {
        vision.setState(VisionState.TAGS);
        shooter.idleRequest();
        shooterHood.idleRequest();
        dyeRotor.idleRequest();
        // Set turret behavior separate climbing
        deploy.stowRequest();
        intake.idleRequest();
        swerve.normalDriveRequest();
        climber.l2HangingRequest();
      }
      case MANUAL_CLIMB_5_RAISING_L3 -> {
        vision.setState(VisionState.TAGS);
        shooter.idleRequest();
        shooterHood.idleRequest();
        dyeRotor.idleRequest();
        // Set turret behavior separate climbing
        deploy.stowRequest();
        intake.idleRequest();
        swerve.normalDriveRequest();
        climber.l3LineupRequest();
      }
      case MANUAL_CLIMB_6_HANGING_L3 -> {
        vision.setState(VisionState.TAGS);
        shooter.idleRequest();
        shooterHood.idleRequest();
        dyeRotor.idleRequest();
        // Set turret behavior separate climbing
        deploy.stowRequest();
        intake.idleRequest();
        swerve.normalDriveRequest();
        climber.l3HangingRequest();
      }
      case AUTOMATIC_CLIMB_1_APPROACH_L1 -> {
        vision.setState(VisionState.TAGS);
        shooter.idleRequest();
        shooterHood.idleRequest();
        dyeRotor.idleRequest();
        // Set turret behavior separate climbing
        deploy.stowRequest();
        intake.idleRequest();
        trailblazer.setActiveSegment(
            ClimbAssist.getApproachClimbAssistSegment(robotPose, ClimbLocation.CLOSEST));
        swerve.climbAssistDriveRequest();
        climber.l1LineupRequest();
      }
      case AUTOMATIC_CLIMB_2_LINEUP_L1 -> {
        vision.setState(VisionState.TAGS);
        shooter.idleRequest();
        shooterHood.idleRequest();
        dyeRotor.idleRequest();
        // Set turret behavior separate climbing
        deploy.stowRequest();
        intake.idleRequest();
        trailblazer.setActiveSegment(
            ClimbAssist.getLineupClimbAssistSegment(robotPose, ClimbLocation.CLOSEST));
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
        climber.l3HangingRequest();
      }
      case CLIMB_7_PREPARE_SCORING_L3 -> {
        vision.setState(VisionState.TAGS);
        shooter.climbScoreRequest(climbLocationIsLeft);
        shooterHood.climbScoreRequest(climbLocationIsLeft);
        dyeRotor.idleRequest();
        turret.climbScoreRequest(climbLocationIsLeft);
        deploy.stowRequest();
        intake.idleRequest();
        swerve.normalDriveRequest();
        climber.l3HangingRequest();
      }
      case CLIMB_8_SCORING_L3 -> {
        vision.setState(VisionState.TAGS);
        shooter.climbScoreRequest(climbLocationIsLeft);
        shooterHood.climbScoreRequest(climbLocationIsLeft);
        dyeRotor.scoreRequest(scoringParameters.distance());
        turret.climbScoreRequest(climbLocationIsLeft);
        deploy.stowRequest();
        intake.shootRequest();
        swerve.normalDriveRequest();
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
        if (intake.getState() == IntakeState.INTAKE) {
          swerve.intakeDriveRequest();
        } else {
          swerve.normalDriveRequest();
        }
      }

      case PREPARE_SCORE, STOP_SHOOTING_SCORE -> {
        smartTurretHoodPrepareScoreRequest();
        if (!DSOptions.USE_TURRET.getAsBoolean()) {
          swerve.turretStuckAimRequest(scoringParameters.turretAngle());
        } else if (intake.getState() == IntakeState.INTAKE) {
          swerve.intakeRateLimitedDriveRequest();
        } else {
          swerve.rateLimitedDriveRequest();
        }
        // isHubActive always logged
      }
      case SCORE -> {
        turret.scoreRequest(scoringParameters.turretAngle());
        shooterHood.scoreRequest(scoringParameters.distance());
        if (!DSOptions.USE_TURRET.getAsBoolean()) {
          swerve.turretStuckAimRequest(scoringParameters.turretAngle());
        } else if (intake.getState() == IntakeState.INTAKE) {
          swerve.intakeRateLimitedDriveRequest();
        } else {
          swerve.rateLimitedDriveRequest();
        }

        if (FeatureFlags.DYE_ROTOR_CLEANUP_MODE.getAsBoolean()
            && intake.getState() == IntakeState.INTAKE) {
          dyeRotor.scoreCleanupRequest(scoringParameters.distance());
        } else {
          dyeRotor.scoreRequest(scoringParameters.distance());
        }

        if (drivingToIntake) {
          deploy.intakeRequest();
        } else {
          deploy.shuffleRequest();
        }
      }
      case PREPARE_FEED -> {
        smartTurretHoodPrepareFeedRequest();
        if (intake.getState() == IntakeState.INTAKE) {
          swerve.intakeDriveRequest();
        } else {
          swerve.normalDriveRequest();
        }
      }
      case FEED -> {
        turret.feedRequest(feedingParameters.turretAngle());
        shooterHood.feedRequest(feedingParameters.distance());
        if (intake.getState() == IntakeState.INTAKE) {
          swerve.intakeDriveRequest();
        } else {
          swerve.normalDriveRequest();
        }

        if (FeatureFlags.DYE_ROTOR_CLEANUP_MODE.getAsBoolean()
            && intake.getState() == IntakeState.INTAKE) {
          dyeRotor.feedCleanupRequest(feedingParameters.distance());
        } else {
          dyeRotor.feedRequest(feedingParameters.distance());
        }
        if (drivingToIntake) {
          deploy.intakeRequest();
        } else {
          deploy.shuffleRequest();
        }
      }

      // Fallback states
      case PREPARE_PRESET_SCORE -> {
        // Automatically update scoring parameters with preset pose
        if (isMoving) {
          shooterHood.idleRequest();
        } else {
          shooterHood.scoreRequest(scoringParameters.distance());
        }
        turret.scoreRequest(scoringParameters.turretAngle());
        if (intake.getState() == IntakeState.INTAKE) {
          swerve.intakeDriveRequest();
        } else {
          swerve.normalDriveRequest();
        }
      }
      case PRESET_SCORE -> {
        // Automatically update scoring parameters with preset pose
        shooterHood.scoreRequest(scoringParameters.distance());
        turret.scoreRequest(scoringParameters.turretAngle());
        if (intake.getState() == IntakeState.INTAKE) {
          swerve.intakeDriveRequest();
        } else {
          swerve.normalDriveRequest();
        }
        if (drivingToIntake) {
          deploy.intakeRequest();
        } else {
          deploy.shuffleRequest();
        }
      }
      case PREPARE_PRESET_FEED -> {
        // TODO: Get turret feed angle
        turret.feedRequest(0);
        if (intake.getState() == IntakeState.INTAKE) {
          swerve.intakeDriveRequest();
        } else {
          swerve.normalDriveRequest();
        }
      }
      case PRESET_FEED -> {
        // TODO: get turret feed angle
        turret.feedRequest(0);
        if (intake.getState() == IntakeState.INTAKE) {
          swerve.intakeDriveRequest();
        } else {
          swerve.normalDriveRequest();
        }
        if (drivingToIntake) {
          deploy.intakeRequest();
        } else {
          deploy.shuffleRequest();
        }
      }
      case AUTOMATIC_CLIMB_1_APPROACH_L1, AUTOMATIC_CLIMB_2_LINEUP_L1 -> {
        turret.climbRequest(robotPose);
        swerve.climbAssistDriveRequest();
      }
      case AUTOMATIC_CLIMB_3_HANGING_L1,
          AUTOMATIC_CLIMB_4_RAISING_L2,
          AUTOMATIC_CLIMB_5_HANGING_L2,
          AUTOMATIC_CLIMB_6_RAISING_L3,
          AUTOMATIC_CLIMB_7_HANGING_L3,
          CLIMB_1_LINEUP_L1_AUTONOMOUS,
          CLIMB_2_RAISING_L1_AUTONOMOUS,
          CLIMB_3_HANGING_L1_AUTONOMOUS,
          CLIMB_4_RELEASE_L1_AUTONOMOUS,
          MANUAL_CLIMB_1_LINEUP_L1,
          MANUAL_CLIMB_2_HANGING_L1,
          MANUAL_CLIMB_3_RAISING_L2,
          MANUAL_CLIMB_4_HANGING_L2,
          MANUAL_CLIMB_5_RAISING_L3,
          MANUAL_CLIMB_6_HANGING_L3 -> {
        turret.climbRequest(robotPose);
      }
      default -> {}
    }
    DogLog.log("RobotManager/Feeding/FeedLocation", feedLocation);
    DogLog.log("RobotManager/Feeding/FeedParameters", feedingParameters);
    DogLog.log("RobotManager/Scoring/ScoringParameters", scoringParameters);

    DogLog.log("RobotManager/Scoring/ScoreTransition/IsHubActive", isHubActive);
    DogLog.log("RobotManager/TimeSinceMatchStart", timeSinceMatchStart);
    DogLog.log("RobotManager/TimeSinceTeleopEnable", teleopTimer.get());

    DogLog.log("RobotManager/TimeUntilNextShift", timeUntilNextShift);
    DogLog.log("RobotManager/HubActive", getIsHubActive());
    DogLog.log("RobotManager/DrivingToIntake", drivingToIntake);

    MechanismVisualizer.log(
        robotPose,
        turret.getAngle(),
        shooterHood.getAngle(),
        deploy.getPosition(),
        climber.getHeight(),
        dyeRotor.getAngle());
  }

  private void smartTurretHoodIdleRequest() {
    // -First, if cameras are offline or we are near a trench, always be idle
    // -Otherwise if we are in our alliance zone, point towards hub
    // -And if we are not in alliance zone, point towards feed pose
    if (!health.isLocalizationHealthy() || !localization.isTrustworthy() || nearTrench) {
      shooterHood.idleRequest();
      turret.idleScoreRequest(scoringParameters.turretAngle());

      DogLog.log("RobotManager/SmartIdle/Status", "NearTrench");
    } else if (FieldUtil.isRobotPastObstacleTowardAllianceZone(robotPose.getTranslation())) {
      if (FeatureFlags.HOOD_ALWAYS_IDLE.getAsBoolean()) {
        shooterHood.idleRequest();
      } else {
        shooterHood.scoreRequest(scoringParameters.distance());
      }
      turret.idleScoreRequest(scoringParameters.turretAngle());

      DogLog.log("RobotManager/SmartIdle/Status", "InAllianceZone");
    } else {
      if (FeatureFlags.HOOD_ALWAYS_IDLE.getAsBoolean()) {
        shooterHood.idleRequest();
      } else {
        shooterHood.feedRequest(feedingParameters.distance());
      }
      turret.idleFeedRequest(feedingParameters.turretAngle());

      DogLog.log("RobotManager/SmartIdle/Status", "NotInAlliance");
    }
  }

  private void smartTurretHoodPrepareScoreRequest() {
    // Turret behavior
    if (FieldUtil.isRobotPastObstacleTowardAllianceZone(robotPose.getTranslation())) {
      DogLog.log("RobotManager/Scoring/SmartPrepareScore/TurretStatus", "InAllianceZone");

      turret.scoreRequest(scoringParameters.turretAngle());
    } else {
      DogLog.log("RobotManager/Scoring/SmartPrepareScore/TurretStatus", "NotInAllianceZone");

      turret.idleScoreRequest(scoringParameters.turretAngle());
    }

    // Hood Behavior
    if (!health.isLocalizationHealthy() || nearTrench) {
      shooterHood.idleRequest();
      DogLog.log("RobotManager/Scoring/SmartPrepareScore/HoodStatus", "NearTrench");
    } else {
      DogLog.log("RobotManager/Scoring/SmartPrepareScore/HoodStatus", "NotNearTrench");

      shooterHood.scoreRequest(scoringParameters.distance());
    }
  }

  private void smartTurretHoodPrepareFeedRequest() {
    // Turret behavior
    if (FieldUtil.isRobotPastObstacleTowardAllianceZone(robotPose.getTranslation())) {
      DogLog.log("RobotManager/Scoring/SmartPrepareScore/TurretStatus", "InAllianceZone");
      turret.feedRequest(feedingParameters.turretAngle());
    } else {
      DogLog.log("RobotManager/Scoring/SmartPrepareScore/TurretStatus", "NotInAllianceZone");

      turret.idleFeedRequest(feedingParameters.turretAngle());
    }

    // Hood Behavior
    if (!health.isLocalizationHealthy() || nearTrench) {
      shooterHood.idleRequest();
      DogLog.log("RobotManager/Scoring/SmartPrepareScore/HoodStatus", "NearTrench");
    } else {
      DogLog.log("RobotManager/Scoring/SmartPrepareScore/HoodStatus", "NotNearTrench");

      shooterHood.feedRequest(feedingParameters.distance());
    }
  }

  public void idleRequest() {
    if (FeatureFlags.STOP_SHOOTING_STATE.getAsBoolean()) {
      if (!getState().isClimbing()) {
        switch (getState()) {
          case SCORE -> setStateFromRequest(RobotState.STOP_SHOOTING_SCORE);
          case PRESET_SCORE -> setStateFromRequest(RobotState.STOP_SHOOTING_PRESET_SCORE);
          case FEED -> setStateFromRequest(RobotState.STOP_SHOOTING_FEED);
          case PRESET_FEED -> setStateFromRequest(RobotState.STOP_SHOOTING_PRESET_FEED);
          default -> setStateFromRequest(RobotState.IDLE);
        }
      }
    } else {
      if (!getState().isClimbing()) {
        setStateFromRequest(RobotState.IDLE);
      }
    }
  }

  public void forceShootRequest() {
    if (!getState().isClimbing() && getState() != RobotState.FORCE_SCORE) {
      setStateFromRequest(RobotState.PREPARE_FORCE_SCORE);
    }
  }

  public void prepareScoreRequest() {
    if (!getState().isClimbing()
        && getState() != RobotState.PRESET_SCORE
        && getState() != RobotState.SCORE) {
      if (!health.isLocalizationHealthy()) {
        setStateFromRequest(RobotState.PREPARE_PRESET_SCORE);
      } else {
        setStateFromRequest(RobotState.PREPARE_SCORE);
      }
    }
  }

  public void prepareFeedRequest() {
    if (!getState().isClimbing()
        && getState() != RobotState.PRESET_FEED
        && getState() != RobotState.FEED) {
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

    // If we are shooting while cancelling a previous intake request, send a new
    // hopper shuffle request
    switch (getState()) {
      case PREPARE_FORCE_SCORE,
          FORCE_SCORE,
          PREPARE_PRESET_SCORE,
          PRESET_SCORE,
          PREPARE_SCORE,
          SCORE,
          PREPARE_PRESET_FEED,
          PRESET_FEED,
          PREPARE_FEED,
          FEED ->
          deploy.shuffleRequest();
      default -> {}
    }
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
      setStateFromRequest(RobotState.AUTOMATIC_CLIMB_1_APPROACH_L1);
    }
  }

  public void stopTeleopAutoClimbAlignment() {
    switch (getState()) {
      case AUTOMATIC_CLIMB_1_APPROACH_L1, AUTOMATIC_CLIMB_2_LINEUP_L1 ->
          setStateFromRequest(RobotState.IDLE);
      default -> {}
    }
  }

  public void manualClimbSequenceForward() {
    switch (getState()) {
      default -> setStateFromRequest(RobotState.MANUAL_CLIMB_1_LINEUP_L1);
      case MANUAL_CLIMB_1_LINEUP_L1, AUTOMATIC_CLIMB_1_APPROACH_L1, AUTOMATIC_CLIMB_2_LINEUP_L1 ->
          setStateFromRequest(RobotState.MANUAL_CLIMB_2_HANGING_L1);
      case MANUAL_CLIMB_2_HANGING_L1, AUTOMATIC_CLIMB_3_HANGING_L1 ->
          setStateFromRequest(RobotState.MANUAL_CLIMB_3_RAISING_L2);
      case MANUAL_CLIMB_3_RAISING_L2, AUTOMATIC_CLIMB_4_RAISING_L2 ->
          setStateFromRequest(RobotState.MANUAL_CLIMB_4_HANGING_L2);
      case MANUAL_CLIMB_4_HANGING_L2, AUTOMATIC_CLIMB_5_HANGING_L2 ->
          setStateFromRequest(RobotState.MANUAL_CLIMB_5_RAISING_L3);
      case MANUAL_CLIMB_5_RAISING_L3, AUTOMATIC_CLIMB_6_RAISING_L3 ->
          setStateFromRequest(RobotState.MANUAL_CLIMB_6_HANGING_L3);
      case MANUAL_CLIMB_6_HANGING_L3, AUTOMATIC_CLIMB_7_HANGING_L3 ->
          setStateFromRequest(RobotState.CLIMB_7_PREPARE_SCORING_L3);
    }
  }

  public void manualClimbSequenceBackwardOrIdleRequest() {
    switch (getState()) {
      default -> {
        idleRequest();
      }
      case CLIMB_1_LINEUP_L1_AUTONOMOUS -> setStateFromRequest(RobotState.IDLE);
      case CLIMB_2_RAISING_L1_AUTONOMOUS ->
          setStateFromRequest(RobotState.CLIMB_1_LINEUP_L1_AUTONOMOUS);
      case CLIMB_3_HANGING_L1_AUTONOMOUS ->
          setStateFromRequest(RobotState.CLIMB_2_RAISING_L1_AUTONOMOUS);

      // This is the last step in the climb sequence, so just go to stowed
      case MANUAL_CLIMB_1_LINEUP_L1, AUTOMATIC_CLIMB_1_APPROACH_L1 ->
          setStateFromRequest(RobotState.IDLE);
      case AUTOMATIC_CLIMB_2_LINEUP_L1 -> setStateFromRequest(RobotState.MANUAL_CLIMB_1_LINEUP_L1);
      case MANUAL_CLIMB_2_HANGING_L1, AUTOMATIC_CLIMB_3_HANGING_L1 ->
          setStateFromRequest(RobotState.MANUAL_CLIMB_1_LINEUP_L1);

      case MANUAL_CLIMB_3_RAISING_L2, AUTOMATIC_CLIMB_4_RAISING_L2 ->
          setStateFromRequest(RobotState.MANUAL_CLIMB_2_HANGING_L1);
      case MANUAL_CLIMB_4_HANGING_L2, AUTOMATIC_CLIMB_5_HANGING_L2 ->
          setStateFromRequest(RobotState.MANUAL_CLIMB_3_RAISING_L2);

      case MANUAL_CLIMB_5_RAISING_L3, AUTOMATIC_CLIMB_6_RAISING_L3 ->
          setStateFromRequest(RobotState.MANUAL_CLIMB_4_HANGING_L2);
      case MANUAL_CLIMB_6_HANGING_L3, AUTOMATIC_CLIMB_7_HANGING_L3 ->
          setStateFromRequest(RobotState.MANUAL_CLIMB_5_RAISING_L3);
      case CLIMB_7_PREPARE_SCORING_L3 -> setStateFromRequest(RobotState.MANUAL_CLIMB_6_HANGING_L3);
      case CLIMB_8_SCORING_L3 -> setStateFromRequest(RobotState.MANUAL_CLIMB_6_HANGING_L3);
    }
  }

  @Override
  protected void collectInputs() {
    robotPose = localization.getPose();
    double robotRotation = robotPose.getRotation().getDegrees();
    vision.setEstimatedPoseAngle(robotRotation);
    turret.setRobotRotationRate(swerve.getFieldRelativeSpeeds().omegaRadiansPerSecond);

    if (!DSOptions.USE_TURRET.getAsBoolean()) {
      vision.calibrateTurretRequest();
      turret.stuckRequest();
      turret.setStuckAngle(vision.getCalibratedTurretAngle().orElse(0.0));
    }

    if (health.isLocalizationHealthy()) {
      climbLocationIsLeft = ClimbLocation.getNearest(robotPose) == ClimbLocation.LEFT;
    } else {
      climbLocationIsLeft = ClimbAssist.getClimbLocation() == ClimbLocation.LEFT;
    }
    var speeds = swerve.getFieldRelativeSpeeds();
    isMoving = MathHelpers.getLinearVelocity(speeds) > 0.2;

    nearTrench =
        FieldUtil.inTrench(robotPose.getTranslation())
            || SwerveAssist.ableToTrenchAssist(robotPose, swerve.getFieldRelativeSpeeds());

    scoringParameters =
        AimParameterUtil.getScoringParameters(
            health.isLocalizationHealthy()
                ? robotPose
                : new Pose2d(
                    FieldUtil.getFallbackScorePoint().getTranslation(), robotPose.getRotation()),
            swerve.getFieldRelativeSpeeds());

    if (!DSOptions.USE_TURRET.getAsBoolean()) {
      scoringParameters =
          AimParameterUtil.getTurretStuckScoringParameters(
              health.isLocalizationHealthy()
                  ? robotPose
                  : new Pose2d(
                      FieldUtil.getFallbackScorePoint().getTranslation(), robotPose.getRotation()),
              turret.getAngle(),
              swerve.getFieldRelativeSpeeds());
    }

    shooter.getScoreTimeOfFlight(scoringParameters.distance());
    feedingParameters =
        AimParameterUtil.getFeedingParameters(
            feedLocation, robotPose, swerve.getFieldRelativeSpeeds());

    timeSinceMatchStart = teleopTimer.get() + FmsUtil.MATCH_TIME_AT_TELEOP_START;

    isHubActive = getIsHubActiveOrNotUsingState();
    timeUntilNextShift = FmsUtil.timeUntilNextShift(timeSinceMatchStart);
    DogLog.log("RobotManager/CurrentShift", FmsUtil.currentShift(timeSinceMatchStart));
    var swerveVector = MathHelpers.getDriveDirection(speeds);
    double driveDirection = swerveVector.getDegrees();
    drivingToIntake =
        intake.getState() == IntakeState.INTAKE
            && MathUtil.isNear(robotRotation, driveDirection, 45, -180, 180)
            && MathHelpers.getLinearVelocity(speeds) > 1e-5;
    isCloseEnoughToHub = getIsCloseEnoughToHub();
  }

  private boolean getIsHubActiveOrNotUsingState() {
    if (!DSOptions.USE_HUB_STATE.get() || DriverStation.isAutonomousEnabled()) {
      return true;
    }

    return getIsHubActive();
  }

  private boolean getIsHubActive() {
    if (FeatureFlags.LOOKAHEAD_SCORING.getAsBoolean()) {
      return FmsUtil.isHubActive(
          timeSinceMatchStart
              + shooter.getScoreTimeOfFlight(scoringParameters.distance())
              + tunableHubStateOffset.get(),
          DSOptions.DEFAULT_WON_AUTO.getAsBoolean());
    }

    return FmsUtil.isHubActive(
        timeSinceMatchStart + tunableHubStateOffset.get(),
        DSOptions.DEFAULT_WON_AUTO.getAsBoolean());
  }

  private boolean getIsCloseEnoughToHub() {
    return scoringParameters.distance() < DISTANCE_TO_HUB_THRESHOLD.get();
  }

  private void logScoringTransition() {
    DogLog.log("Debug/TurretScoreTolerance", scoringParameters.turretTolerance());
    DogLog.log("RobotManager/Scoring/ScoreTransition/ShooterAtGoal", shooter.atGoal());
    DogLog.log(
        "RobotManager/Scoring/ScoreTransition/LocalizationTrustworthy",
        localization.isTrustworthy());
    DogLog.log(
        "RobotManager/Scoring/ScoreTransition/InAllianceZone",
        FieldUtil.isRobotInAllianceZone(robotPose.getTranslation()));
    DogLog.log("RobotManager/Scoring/ScoreTransition/DyeRotorNotJammed", !dyeRotor.isJammed());
    DogLog.log(
        "RobotManager/Scoring/ScoreTransition/TurretAtGoal",
        turret.atGoal(scoringParameters.turretTolerance()));
    DogLog.log("RobotManager/Scoring/ScoreTransition/ShooterHoodAtGoal", shooterHood.atGoal());
    DogLog.log("RobotManager/Scoring/ScoreTransition/CloseEnoughToHub", isCloseEnoughToHub);
  }

  private void logFeedTransition() {
    DogLog.log("Debug/TurretFeedTolerance", feedingParameters.turretTolerance());

    DogLog.log("RobotManager/Feeding/FeedTransition/ShooterAtGoal", shooter.atGoal());
    DogLog.log(
        "RobotManager/Feeding/FeedTransition/LocalizationHealthy", health.isLocalizationHealthy());
    DogLog.log(
        "RobotManager/Feeding/FeedTransition/InNoFeedZone",
        !FieldUtil.isRobotInNoFeedZone(TurretCalculator.getTurretPose(robotPose)));
    DogLog.log("RobotManager/Feeding/FeedTransition/DyeRotorNotJammed", !dyeRotor.isJammed());
    DogLog.log(
        "RobotManager/Feeding/FeedTransition/TurretAtGoal",
        turret.atGoal(feedingParameters.turretTolerance()));
    DogLog.log("RobotManager/Feeding/FeedTransition/ShooterHoodAtGoal", shooterHood.atGoal());
  }

  // TODO: Every time the driver/operator left/right trigger changes, run this function with the
  // full state of their requested intake + deploy state
  public void teleopDeployRequest(
      boolean operatorWantsForceStow,
      boolean driverWantsIntake,
      boolean driverWantsHubScore,
      boolean driverWantsFeed,
      boolean operatorWantsHubScore,
      boolean operatorWantsFeed) {
    if (operatorWantsForceStow) {
      deploy.stowRequest();
      intake.idleRequest();
      return;
    }

    if (driverWantsIntake) {
      // TODO: This should check if driver also wants to score, and do smart shuffle stuff based on
      // drive vector
      deploy.intakeRequest();
      intake.intakeRequest();
      return;
    }

    if (driverWantsHubScore || operatorWantsHubScore || driverWantsFeed || operatorWantsFeed) {
      deploy.shuffleRequest();
      intake.shootRequest();
      return;
    }

    deploy.intakeRequest();
    intake.idleRequest();
  }
}
