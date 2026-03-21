package frc.robot.robot_manager;

import com.team581.autos.Point;
import com.team581.math.MathHelpers;
import com.team581.swerve.SwerveAssist;
import com.team581.trailblazer.Trailblazer;
import com.team581.util.FeedLocation;
import com.team581.util.FieldUtil;
import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Hardware;
import frc.robot.climber.ClimbLocation;
import frc.robot.climber.GenericClimber;
import frc.robot.cluster_map.ClusterMap;
import frc.robot.config.DSOptions;
import frc.robot.config.FeatureFlags;
import frc.robot.deploy.Deploy;
import frc.robot.dye_rotor.DyeRotor;
import frc.robot.health.HealthManager;
import frc.robot.hub_activity.HubActivity;
import frc.robot.intake.GenericIntake;
import frc.robot.localization.Localization;
import frc.robot.power_manager.PowerManager;
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
  public final DyeRotor dyeRotor;
  public final Deploy deploy;
  private final Turret turret;
  private final GenericIntake intake;
  private final Vision vision;
  public final XboxController driverController;
  private final HealthManager health;
  private final HubActivity hubActivity;
  private final Trailblazer trailblazer;
  public final ClusterMap clusterMap;
  private final GenericClimber climber;

  private final PowerManager powerManager;

  private Pose2d robotPose = Pose2d.kZero;
  private boolean nearTrench = false;

  private boolean climbLocationIsLeft = true;

  private AimingParameters scoringParameters = new AimingParameters(0, 0, 0, 0, 0);
  private AimingParameters feedingParameters = new AimingParameters(0, 0, 0, 0, 0);
  private static final double PRESET_FEED_DISTANCE = 0.0;
  private boolean isMoving = false;
  private boolean drivingToIntake = false;
  private boolean driverWantsToIntake = false;
  private boolean trenchOverride = false;

  private boolean isInSafeScoringLocation = false;
  private boolean isInAllianceZone = false;
  private boolean isInSafeFeedingLocation = true;

  private FeedLocation feedLocation = FeedLocation.CLOSEST;
  private AimingParameters fallbackFeedingParameters = new AimingParameters(0, 0, 0, 0, 0);

  public RobotManager(
      ShooterHood shooterHood,
      Localization localization,
      Swerve swerve,
      Shooter shooter,
      DyeRotor dyeRotor,
      Turret turret,
      GenericIntake intake,
      Deploy deploy,
      Vision vision,
      XboxController driverController,
      HealthManager health,
      HubActivity hubActivity,
      Trailblazer trailblazer,
      GenericClimber climber,
      ClusterMap clusterMap,
      Hardware hardware,
      PowerManager powerManager) {
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
    this.hubActivity = hubActivity;
    this.trailblazer = trailblazer;
    this.clusterMap = clusterMap;
    this.climber = climber;
    this.powerManager = powerManager;

    this.hardware = hardware;
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
          dyeRotor.isReset() ? RobotState.IDLE : currentState;
      case PREPARE_FORCE_SCORE -> {
        if (turret.atGoal(scoringParameters)
            && shooter.atGoalDebounced()
            && !dyeRotor.isJammed()
            && shooterHood.atGoal()) {
          yield RobotState.FORCE_SCORE;
        }
        yield currentState;
      }
      case IDLE -> currentState;

      case PREPARE_PRESET_SCORE, PRESET_SCORE ->
          !isMoving
                  && ((shooter.atGoalDebounced()
                          && !dyeRotor.isJammed()
                          && turret.atGoal(scoringParameters)
                          && shooterHood.atGoal())
                      || hubActivity.ableToForceScoreTransitionEndOfActiveHub())
              ? RobotState.PRESET_SCORE
              : RobotState.PREPARE_PRESET_SCORE;
      case CLIMB_7_PREPARE_SCORING_L3 -> {
        if (shooter.atGoalDebounced()
            && turret.atGoal(1.0)
            && shooterHood.atGoal()
            && !dyeRotor.isJammed()) {
          yield RobotState.CLIMB_8_SCORING_L3;
        }
        yield currentState;
      }
      case PREPARE_PRESET_FEED, PRESET_FEED ->
          shooter.atGoalDebounced()
                  && !dyeRotor.isJammed()
                  && turret.atGoal(1, feedingParameters.upcomingTurretAngle())
                  && shooterHood.atGoal()
              ? RobotState.PRESET_FEED
              : RobotState.PREPARE_PRESET_FEED;
      case PREPARE_SCORE -> {
        logScoringTransition();

        if (!isInAllianceZone) {
          if (DriverStation.isTeleop()) {
            yield RobotState.PREPARE_FEED;
          }
          yield currentState;
        }

        if ((turret.atGoal(scoringParameters)
                && shooter.atGoalDebounced()
                && shooterHood.atGoal()
                && swerve.isAimed()
                && localization.isTrustworthy()
                && localization.imu.isFlatDebounced()
                && hubActivity.getTOFBasedHubActive()
                && isInSafeScoringLocation
                && !nearTrench)
            || (hubActivity.ableToForceScoreTransitionEndOfActiveHub()
                && shooter.atGoalDebounced())) {
          yield RobotState.SCORE;
        }
        yield currentState;
      }
      case SCORE -> {
        logScoringTransition();

        if (!isInAllianceZone) {
          if (DriverStation.isTeleop()) {
            yield RobotState.PREPARE_FEED;
          }
          yield currentState;
        }

        if (!hubActivity.getTOFBasedHubActive()) {
          yield RobotState.STOP_SHOOTING_SCORE;
        }

        if ((!FeatureFlags.CANCEL_IN_PROGRESS_SHOT.getAsBoolean()
                || (localization.isTrustworthy()
                    && turret.atGoal(scoringParameters)
                    && shooterHood.atGoal()
                    && swerve.isAimed()
                    && localization.imu.isFlatDebounced()
                    && localization.isTrustworthy()
                    && isInSafeScoringLocation)
                || (hubActivity.ableToForceScoreTransitionEndOfActiveHub()
                    && shooter.atGoalDebounced()))
            && !nearTrench) {
          yield currentState;
        }

        yield RobotState.PREPARE_SCORE;
      }
      case PREPARE_FEED -> {
        logFeedTransition();

        if (isInAllianceZone) {
          yield RobotState.PREPARE_SCORE;
        }

        if (shooter.atGoalDebounced()
            && isInSafeFeedingLocation
            && localization.imu.isFlatDebounced()
            && turret.atGoal(feedingParameters)
            && shooterHood.atGoal()
            && health.isLocalizationHealthy()
            && !nearTrench) {

          yield RobotState.FEED;
        } else {
          yield currentState;
        }
      }
      case FEED -> {
        if (isInAllianceZone) {
          yield RobotState.PREPARE_SCORE;
        }

        logFeedTransition();
        if (!FeatureFlags.CANCEL_IN_PROGRESS_SHOT.getAsBoolean()
            || (isInSafeFeedingLocation
                && turret.atGoal(feedingParameters)
                && localization.imu.isFlatDebounced()
                && shooterHood.atGoal()
                && health.isLocalizationHealthy()
                && !nearTrench)) {

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
        powerManager.idleRequest();
        vision.setState(VisionState.HUB_TAGS);
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
        turret.scoreRequest(
            scoringParameters.turretAngle(), scoringParameters.turretFeedForwardRadians());
        swerve.normalDriveRequest();
        climber.stowRequest();
      }
      case FORCE_SCORE -> {
        vision.setState(VisionState.TAGS);
        shooter.scoreRequest(scoringParameters.distance());
        shooterHood.scoreRequest(scoringParameters.distance());
        dyeRotor.scoreRequest(scoringParameters.distance());
        turret.scoreRequest(
            scoringParameters.turretAngle(), scoringParameters.turretFeedForwardRadians());
        if (intake.getState().isIntaking()) {
          intake.shootThenIntakeRequest();
        } else {
          intake.shootRequest();
        }
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
        swerve.rateLimitedDriveRequest();
        climber.stowRequest();
      }
      case FEED -> {
        vision.setState(VisionState.TAGS);
        shooter.feedRequest(feedingParameters.distance());
        shooterHood.feedRequest(feedingParameters.distance());
        dyeRotor.feedRequest(feedingParameters.distance());
        turret.feedRequest(
            feedingParameters.turretAngle(), feedingParameters.turretFeedForwardRadians());
        if (intake.getState().isIntaking()) {
          intake.shootThenIntakeRequest();
        } else {
          intake.shootRequest();
        }
        swerve.rateLimitedDriveRequest();
        climber.stowRequest();
      }
      case STOP_SHOOTING_FEED -> {
        vision.setState(VisionState.TAGS);
        shooter.feedRequest(feedingParameters.distance());
        shooterHood.feedRequest(feedingParameters.distance());
        dyeRotor.resetToIdleRequest();
        turret.feedRequest(
            feedingParameters.turretAngle(), feedingParameters.turretFeedForwardRadians());
        deploy.stopShootingRequest();
        intake.stopShootingRequest();
        swerve.normalDriveRequest();
        climber.stowRequest();
      }
      case PREPARE_SCORE -> {
        powerManager.shootingRequest();

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
        powerManager.shootingRequest();

        vision.setState(VisionState.HUB_TAGS);
        shooter.scoreRequest(scoringParameters.distance());
        shooterHood.scoreRequest(scoringParameters.distance());
        dyeRotor.scoreRequest(scoringParameters.distance());
        turret.scoreRequest(
            scoringParameters.turretAngle(), scoringParameters.turretFeedForwardRadians());
        if (intake.getState().isIntaking()) {
          intake.shootThenIntakeRequest();
        } else {
          intake.shootRequest();
        }
        swerve.rateLimitedDriveRequest();
        climber.stowRequest();
      }
      case STOP_SHOOTING_SCORE -> {
        vision.setState(VisionState.HUB_TAGS);
        shooter.scoreRequest(scoringParameters.distance());
        shooterHood.scoreRequest(scoringParameters.distance());
        dyeRotor.resetToIdleRequest();
        turret.scoreRequest(
            scoringParameters.turretAngle(), scoringParameters.turretFeedForwardRadians());
        deploy.stopShootingRequest();
        intake.stopShootingRequest();
        swerve.rateLimitedDriveRequest();
        climber.stowRequest();
      }
      case PREPARE_PRESET_FEED -> {
        vision.setState(VisionState.TAGS);
        shooter.feedRequest(PRESET_FEED_DISTANCE);
        shooterHood.feedRequest(PRESET_FEED_DISTANCE);
        dyeRotor.idleRequest();
        turret.feedRequest(
            fallbackFeedingParameters.turretAngle(),
            fallbackFeedingParameters.turretFeedForwardRadians());
        // Deploy is controlled separately
        // Intake is controlled separately
        swerve.normalDriveRequest();
        climber.stowRequest();
      }
      case PRESET_FEED -> {
        vision.setState(VisionState.TAGS);
        shooter.feedRequest(PRESET_FEED_DISTANCE);
        shooterHood.feedRequest(PRESET_FEED_DISTANCE);

        dyeRotor.feedRequest(fallbackFeedingParameters.distance());
        turret.feedRequest(
            fallbackFeedingParameters.turretAngle(),
            fallbackFeedingParameters.turretFeedForwardRadians());

        if (intake.getState().isIntaking()) {
          intake.shootThenIntakeRequest();
        } else {
          intake.shootRequest();
        }
        swerve.normalDriveRequest();
        climber.stowRequest();
      }
      case STOP_SHOOTING_PRESET_FEED -> {
        vision.setState(VisionState.TAGS);
        shooter.feedRequest(PRESET_FEED_DISTANCE);
        shooterHood.feedRequest(PRESET_FEED_DISTANCE);
        dyeRotor.resetToIdleRequest();

        // TODO: Update to use fallback feeding parameters
        turret.feedRequest(
            fallbackFeedingParameters.turretAngle(),
            fallbackFeedingParameters.turretFeedForwardRadians());

        deploy.stopShootingRequest();
        intake.stopShootingRequest();
        swerve.normalDriveRequest();
        climber.stowRequest();
      }
      case PREPARE_PRESET_SCORE -> {
        vision.setState(VisionState.TAGS);
        shooter.scoreRequest(scoringParameters.distance());
        shooterHood.scoreRequest(scoringParameters.distance());
        dyeRotor.idleRequest();
        turret.scoreRequest(
            scoringParameters.turretAngle(), scoringParameters.turretFeedForwardRadians());
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
        turret.scoreRequest(
            scoringParameters.turretAngle(), scoringParameters.turretFeedForwardRadians());
        if (intake.getState().isIntaking()) {
          intake.shootThenIntakeRequest();
        } else {
          intake.shootRequest();
        }
        swerve.normalDriveRequest();
        climber.stowRequest();
      }
      case STOP_SHOOTING_PRESET_SCORE -> {
        vision.setState(VisionState.TAGS);
        shooter.scoreRequest(scoringParameters.distance());
        shooterHood.scoreRequest(scoringParameters.distance());
        dyeRotor.resetToIdleRequest();
        turret.scoreRequest(
            scoringParameters.turretAngle(), scoringParameters.turretFeedForwardRadians());
        intake.stopShootingRequest();
        deploy.stopShootingRequest();
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
        turret.climbScoreRequest(climbLocationIsLeft, 0);
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
        turret.climbScoreRequest(climbLocationIsLeft, 0);
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
        if (intake.getState().isIntaking()) {
          swerve.intakeDriveRequest();
        } else {
          swerve.normalDriveRequest();
        }
      }

      case PREPARE_SCORE, STOP_SHOOTING_SCORE -> {
        smartTurretHoodPrepareScoreRequest();
        if (!DSOptions.USE_TURRET.getAsBoolean()) {
          swerve.turretStuckAimRequest(
              scoringParameters.turretAngle(), scoringParameters.turretFeedForwardRadians());
        } else if (intake.getState().isIntaking()) {
          swerve.intakeRateLimitedDriveRequest();
        } else {
          swerve.rateLimitedDriveRequest();
        }
        // isHubActive always logged
      }
      case SCORE -> {
        turret.scoreRequest(
            scoringParameters.turretAngle(), scoringParameters.turretFeedForwardRadians());
        shooterHood.scoreRequest(scoringParameters.distance());
        if (!DSOptions.USE_TURRET.getAsBoolean()) {
          swerve.turretStuckAimRequest(
              scoringParameters.turretAngle(), scoringParameters.turretFeedForwardRadians());
        } else if (intake.getState().isIntaking()) {
          swerve.intakeRateLimitedDriveRequest();
        } else {
          swerve.rateLimitedDriveRequest();
        }

        dyeRotor.scoreRequest(scoringParameters.distance());

        if (drivingToIntake) {
          deploy.intakeRequest();
        } else {
          deploy.shuffleRequest();
        }
      }
      case PREPARE_FEED -> {
        smartTurretHoodPrepareFeedRequest();
        if (!DSOptions.USE_TURRET.getAsBoolean()) {
          swerve.turretStuckAimRequest(
              feedingParameters.turretAngle(), feedingParameters.turretFeedForwardRadians());
        } else if (intake.getState().isIntaking()) {
          swerve.intakeRateLimitedDriveRequest();
        } else {
          swerve.rateLimitedDriveRequest();
        }
      }
      case FEED -> {
        turret.feedRequest(
            feedingParameters.turretAngle(), feedingParameters.turretFeedForwardRadians());
        shooterHood.feedRequest(feedingParameters.distance());
        if (!DSOptions.USE_TURRET.getAsBoolean()) {
          swerve.turretStuckAimRequest(
              feedingParameters.turretAngle(), feedingParameters.turretFeedForwardRadians());
        } else if (intake.getState().isIntaking()) {
          swerve.intakeRateLimitedDriveRequest();
        } else {
          swerve.rateLimitedDriveRequest();
        }

        dyeRotor.feedRequest(feedingParameters.distance());

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
        turret.scoreRequest(
            scoringParameters.turretAngle(), scoringParameters.turretFeedForwardRadians());
        if (intake.getState().isIntaking()) {
          swerve.intakeDriveRequest();
        } else {
          swerve.normalDriveRequest();
        }
      }
      case PRESET_SCORE -> {
        // Automatically update scoring parameters with preset pose
        shooterHood.scoreRequest(scoringParameters.distance());
        turret.scoreRequest(
            scoringParameters.turretAngle(), scoringParameters.turretFeedForwardRadians());
        if (intake.getState().isIntaking()) {
          swerve.intakeDriveRequest();
        } else {
          swerve.normalDriveRequest();
        }

        if (drivingToIntake) {
          deploy.intakeRequest();
        } else {
          deploy.shuffleRequest();
        }

        if (shooter.atGoal()) {
          dyeRotor.scoreRequest(scoringParameters.distance());
        } else {
          dyeRotor.scoreSlowRequest();
        }
      }
      case PREPARE_PRESET_FEED -> {
        turret.feedRequest(
            fallbackFeedingParameters.turretAngle(),
            fallbackFeedingParameters.turretFeedForwardRadians());
        if (intake.getState().isIntaking()) {
          swerve.intakeDriveRequest();
        } else {
          swerve.normalDriveRequest();
        }
      }
      case PRESET_FEED -> {
        turret.feedRequest(
            fallbackFeedingParameters.turretAngle(),
            fallbackFeedingParameters.turretFeedForwardRadians());
        if (intake.getState().isIntaking()) {
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
        // TODO: Use actual feed forward
        turret.climbRequest(robotPose, 0);
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
          MANUAL_CLIMB_6_HANGING_L3 -> // TODO: Use actual feed forward
          turret.climbRequest(robotPose, 0);
      default -> {}
    }
    DogLog.log("RobotManager/Feeding/FeedLocation", feedLocation);
    DogLog.log("RobotManager/Feeding/FeedParameters", feedingParameters);
    DogLog.log("RobotManager/Scoring/ScoringParameters", scoringParameters);

    DogLog.log("RobotManager/DrivingToIntake", drivingToIntake);

    DogLog.log("RobotManager/gpDetection/Dye", dyeRotor.velocityDetectsGp());
    DogLog.log("RobotManager/gpDetection/Shooter", shooter.currentDetectsGp());
    DogLog.log("RobotManager/gpDetection/both", detectingGp());

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
      turret.idleScoreRequest(
          scoringParameters.turretAngle(), scoringParameters.turretFeedForwardRadians());

      DogLog.log("RobotManager/SmartIdle/Status", "NearTrench");
    } else if (FieldUtil.isRobotPastObstacleTowardAllianceZone(robotPose.getTranslation())) {
      shooterHood.idleRequest();
      turret.idleScoreRequest(
          scoringParameters.turretAngle(), scoringParameters.turretFeedForwardRadians());

      DogLog.log("RobotManager/SmartIdle/Status", "InAllianceZone");
    } else if (DriverStation.isAutonomous()) {
      shooterHood.idleRequest();
      turret.idleScoreRequest(
          scoringParameters.turretAngle(), scoringParameters.turretFeedForwardRadians());

      DogLog.log("RobotManager/SmartIdle/Status", "UseHubForAuto");
    } else {
      shooterHood.idleRequest();
      turret.idleFeedRequest(
          feedingParameters.turretAngle(), feedingParameters.turretFeedForwardRadians());

      DogLog.log("RobotManager/SmartIdle/Status", "NotInAlliance");
    }
  }

  private void smartTurretHoodPrepareScoreRequest() {
    // Turret behavior
    if (FieldUtil.isRobotPastObstacleTowardAllianceZone(robotPose.getTranslation())) {
      DogLog.log("RobotManager/Scoring/SmartPrepareScore/TurretStatus", "InAllianceZone");

      turret.scoreRequest(
          scoringParameters.turretAngle(), scoringParameters.turretFeedForwardRadians());
    } else {
      DogLog.log("RobotManager/Scoring/SmartPrepareScore/TurretStatus", "NotInAllianceZone");

      turret.idleScoreRequest(
          scoringParameters.turretAngle(), scoringParameters.turretFeedForwardRadians());
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
      turret.feedRequest(
          feedingParameters.turretAngle(), feedingParameters.turretFeedForwardRadians());
    } else {
      DogLog.log("RobotManager/Scoring/SmartPrepareScore/TurretStatus", "NotInAllianceZone");

      turret.idleFeedRequest(
          feedingParameters.turretAngle(), feedingParameters.turretFeedForwardRadians());
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
    if (!getState().isClimbing()) {
      switch (getState()) {
        case SCORE -> setStateFromRequest(RobotState.STOP_SHOOTING_SCORE);
        case PRESET_SCORE -> setStateFromRequest(RobotState.STOP_SHOOTING_PRESET_SCORE);
        case FEED -> setStateFromRequest(RobotState.STOP_SHOOTING_FEED);
        case PRESET_FEED -> setStateFromRequest(RobotState.STOP_SHOOTING_PRESET_FEED);
        default -> setStateFromRequest(RobotState.IDLE);
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

  public void prepareScoreOrFeedRequest() {
    if (isInAllianceZone) {
      prepareScoreRequest();
    } else {
      prepareFeedRequest();
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

  public void setTrenchOverrideRequest(boolean trenchOverride) {
    this.trenchOverride = trenchOverride;
  }

  public void setDriverWantsIntake(boolean wantsIntake) {
    driverWantsToIntake = wantsIntake;
    if (driverWantsToIntake) {
      switch (getState()) {
        case SCORE, FEED, FORCE_SCORE, PRESET_SCORE, PRESET_FEED -> intake.shootThenIntakeRequest();
        default -> intake.intakeRequest();
      }
    } else {
      switch (getState()) {
        case SCORE, FEED, FORCE_SCORE, PRESET_SCORE, PRESET_FEED -> intake.shootRequest();
        default -> intake.idleRequest();
      }
    }
  }

  public void stowDeployRequest() {
    deploy.stowRequest();
    if (driverWantsToIntake) {
      intake.intakeRequest();
    } else {
      intake.idleRequest();
    }
  }

  public void intakeAutoRequest() {
    intake.intakeAutoRequest();
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

  public boolean detectingGp() {
    return dyeRotor.velocityDetectsGp() && shooter.currentDetectsGp();
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

  public void homeDeployInAutoRequest() {
    if (!getState().isClimbing()) {
      deploy.homeInAutoRequest();
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
      default -> idleRequest();
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
    hubActivity.updateShooterScoringTOF(shooter.getScoreTimeOfFlight(scoringParameters.distance()));

    robotPose = localization.getPose();
    double robotRotation = robotPose.getRotation().getDegrees();
    clusterMap.setDeployFullyExtended(deploy.isFullyExtended());
    vision.setEstimatedPoseAngle(robotRotation);

    if (!DSOptions.USE_TURRET.getAsBoolean()) {
      turret.stuckRequest();
    }

    if (health.isLocalizationHealthy()) {
      climbLocationIsLeft = ClimbLocation.getNearest(robotPose) == ClimbLocation.LEFT;
    } else {
      climbLocationIsLeft = ClimbAssist.getClimbLocation() == ClimbLocation.LEFT;
    }
    var speeds = swerve.getFieldRelativeSpeeds();
    isMoving = MathHelpers.getLinearVelocity(speeds) > 0.2;

    // If using clamped points FF we are using the HOME FIELD
    nearTrench =
        !trenchOverride
            && ((Point.CLAMPED_POINTS_FEATURE_FLAG.getAsBoolean()
                    ? FieldUtil.inHomeFieldTrench(robotPose.getTranslation())
                    : FieldUtil.inTrench(robotPose.getTranslation()))
                || SwerveAssist.ableToTrenchAssist(robotPose, swerve.getFieldRelativeSpeeds()));

    scoringParameters =
        AimParameterUtil.getScoringParameters(
            health.isLocalizationHealthy()
                ? robotPose
                : new Pose2d(
                    FieldUtil.getFallbackScorePoint().getTranslation(), robotPose.getRotation()),
            speeds);

    if (!DSOptions.USE_TURRET.getAsBoolean()) {
      scoringParameters =
          AimParameterUtil.getTurretStuckScoringParameters(
              health.isLocalizationHealthy()
                  ? robotPose
                  : new Pose2d(
                      FieldUtil.getFallbackScorePoint().getTranslation(), robotPose.getRotation()),
              turret.getAngle(),
              speeds);
    }

    feedingParameters =
        AimParameterUtil.getFeedingParameters(
            feedLocation, robotPose, swerve.getFieldRelativeSpeeds());

    fallbackFeedingParameters =
        AimParameterUtil.getFallbackFeedingParameters(robotPose.getRotation());

    if (!DSOptions.USE_TURRET.getAsBoolean()) {
      feedingParameters =
          AimParameterUtil.getTurretStuckFeedingParameters(
              feedLocation, robotPose, turret.getAngle(), speeds);
    }

    var swerveVector = MathHelpers.getDriveDirection(speeds);
    double driveDirection = swerveVector.getDegrees();
    drivingToIntake =
        intake.getState().isIntaking()
            && MathUtil.isNear(robotRotation, driveDirection, 120.0, -180, 180)
            && MathHelpers.getLinearVelocity(speeds) > 1e-5;
    isInAllianceZone =
        !health.isLocalizationHealthy()
            ? hubActivity.getTOFBasedHubActive()
            : FieldUtil.isRobotPastObstacleTowardAllianceZone(
                TurretCalculator.getTurretPose(robotPose).getTranslation());

    isInSafeScoringLocation =
        !health.isLocalizationHealthy()
            || !FieldUtil.isScorePathObstructed(robotPose.getTranslation());
    isInSafeFeedingLocation =
        !health.isLocalizationHealthy()
            || !FieldUtil.isFeedPathObstructed(
                robotPose.getTranslation(), feedLocation.getTranslation(robotPose));

    DogLog.log("RobotManager/Scoring/IsInSafeScoringLocation", isInSafeScoringLocation);
    DogLog.log("RobotManager/Feeding/IsInSafeFeedingLocation", isInSafeFeedingLocation);
  }

  private void logScoringTransition() {
    DogLog.log("RobotManager/Scoring/ScoreTransition/InAllianceZone", isInAllianceZone);
    DogLog.log(
        "RobotManager/Scoring/ScoreTransition/TurretAtGoal", turret.atGoal(scoringParameters));

    DogLog.log("RobotManager/Scoring/ScoreTransition/SwerveAimed", swerve.isAimed());

    DogLog.log("RobotManager/Scoring/ScoreTransition/ShooterAtGoal", shooter.atGoalDebounced());
    DogLog.log("RobotManager/Scoring/ScoreTransition/ShooterHoodAtGoal", shooterHood.atGoal());
    DogLog.log(
        "RobotManager/Scoring/ScoreTransition/LocalizationTrustworthy",
        localization.isTrustworthy());
    DogLog.log("RobotManager/Scoring/ScoreTransition/ImuFlat", localization.imu.isFlatDebounced());

    DogLog.log(
        "RobotManager/Scoring/ScoreTransition/HubActive", hubActivity.getTOFBasedHubActive());
    DogLog.log("RobotManager/Scoring/ScoreTransition/IsInScoringZone", isInSafeScoringLocation);
    DogLog.log("RobotManager/Scoring/ScoreTransition/NotNearTrench", !nearTrench);
  }

  private void logFeedTransition() {
    DogLog.log("RobotManager/Feeding/FeedTransition/NotInAllianceZone", !isInAllianceZone);
    DogLog.log("RobotManager/Feeding/FeedTransition/ShooterAtGoal", shooter.atGoalDebounced());
    DogLog.log("RobotManager/Feeding/FeedTransition/SafeFeedLocation", isInSafeFeedingLocation);
    DogLog.log(
        "RobotManager/Feeding/FeedTransition/TurretAtGoal", turret.atGoal(feedingParameters));
    DogLog.log("RobotManager/Feeding/FeedTransition/SwerveAimed", swerve.isAimed());

    DogLog.log("RobotManager/Feeding/FeedTransition/ShooterHoodAtGoal", shooterHood.atGoal());
    DogLog.log(
        "RobotManager/Feeding/FeedTransition/LocalizationHealthy", health.isLocalizationHealthy());
    DogLog.log("RobotManager/Feeding/FeedTransition/ImuFlat", localization.imu.isFlatDebounced());
    DogLog.log("RobotManager/Feeding/ScoreTransition/NotNearTrench", !nearTrench);
  }
}
