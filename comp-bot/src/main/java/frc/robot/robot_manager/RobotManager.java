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
import frc.robot.cluster_map.ClusterMap;
import frc.robot.config.FeatureFlags;
import frc.robot.health.HealthManager;
import frc.robot.hub_activity.HubActivity;
import frc.robot.localization.Localization;
import frc.robot.power_manager.PowerManager;
import frc.robot.robot_manager.hopper_manager.HopperManager;
import frc.robot.shooter.Shooter;
import frc.robot.shooter_hood.ShooterHood;
import frc.robot.swerve.Swerve;
import frc.robot.util.AimParameterUtil;
import frc.robot.util.AimParameterUtil.AimingParameters;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.vision.Vision;
import frc.robot.vision.VisionState;

public class RobotManager extends StateMachineSubsystem<RobotState> {
  public final HopperManager hopperManager;
  public final Localization localization;
  public final Swerve swerve;
  public final Hardware hardware;
  private final ShooterHood shooterHood;
  private final Shooter shooter;
  private final Vision vision;
  public final XboxController driverController;
  private final HealthManager health;
  private final HubActivity hubActivity;
  private final Trailblazer trailblazer;
  public final ClusterMap clusterMap;

  private final PowerManager powerManager;

  private Pose2d robotPose = Pose2d.kZero;
  private boolean nearTrench = false;

  private boolean climbLocationIsLeft = true;

  private AimingParameters scoringParameters = new AimingParameters(0, 0, 0);
  private AimingParameters feedingParameters = new AimingParameters(0, 0, 0);
  private static final double PRESET_FEED_DISTANCE = 0.0;
  private boolean isMoving = false;
  private boolean drivingToIntake = false;
  private boolean driverWantsToIntake = false;
  private boolean trenchOverride = false;

  private boolean isInSafeScoringLocation = false;
  private boolean isInAllianceZone = false;
  private boolean isInSafeFeedingLocation = true;

  private FeedLocation feedLocation = FeedLocation.CLOSEST;
  private AimingParameters fallbackFeedingParameters = new AimingParameters(0, 0, 0);

  public RobotManager(
      HopperManager hopperManager,
      ShooterHood shooterHood,
      Localization localization,
      Swerve swerve,
      Shooter shooter,
      Vision vision,
      XboxController driverController,
      HealthManager health,
      HubActivity hubActivity,
      Trailblazer trailblazer,
      ClusterMap clusterMap,
      Hardware hardware,
      PowerManager powerManager) {
    super(SubsystemPriority.ROBOT_MANAGER, RobotState.IDLE);
    this.hopperManager = hopperManager;
    this.shooterHood = shooterHood;
    this.localization = localization;
    this.swerve = swerve;
    this.shooter = shooter;
    this.vision = vision;
    this.driverController = driverController;
    this.health = health;
    this.hubActivity = hubActivity;
    this.trailblazer = trailblazer;
    this.clusterMap = clusterMap;

    this.hardware = hardware;
    this.powerManager = powerManager;
  }

  @Override
  protected RobotState getNextState(RobotState currentState) {
    return switch (currentState) {
      // No auto transitions for these states
      case UNJAM, FORCE_SCORE -> currentState;
      case STOP_SHOOTING_SCORE,
          STOP_SHOOTING_PRESET_SCORE,
          STOP_SHOOTING_PRESET_FEED,
          STOP_SHOOTING_FEED ->
          RobotState.IDLE;
      case PREPARE_FORCE_SCORE -> {
        if (shooter.atGoalDebounced() && shooterHood.atGoal()) {
          yield RobotState.FORCE_SCORE;
        }
        yield currentState;
      }
      case IDLE -> currentState;

      case PREPARE_PRESET_SCORE, PRESET_SCORE ->
          !isMoving
                  && ((shooter.atGoalDebounced() && shooterHood.atGoal())
                      || hubActivity.ableToForceScoreTransitionEndOfActiveHub())
              ? RobotState.PRESET_SCORE
              : RobotState.PREPARE_PRESET_SCORE;
      case PREPARE_PRESET_FEED, PRESET_FEED ->
          shooter.atGoalDebounced() && shooterHood.atGoal()
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

        if ((shooter.atGoalDebounced()
                && shooterHood.atGoal()
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
                    && shooterHood.atGoal()
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
                && localization.imu.isFlatDebounced()
                && shooterHood.atGoal()
                && health.isLocalizationHealthy()
                && !nearTrench)) {

          yield currentState;
        } else {

          yield RobotState.PREPARE_FEED;
        }
      }
      default -> currentState;
    };
  }

  @Override
  protected void afterTransition(RobotState newState) {
    switch (newState) {
      case IDLE -> {
        vision.setState(VisionState.HUB_TAGS);
        shooter.idleRequest();
        // Set hood behavior separately while idling
        swerve.normalDriveRequest();
      }
      case PREPARE_FORCE_SCORE -> {
        vision.setState(VisionState.TAGS);
        shooter.scoreRequest(scoringParameters.distance());
        shooterHood.scoreRequest(scoringParameters.distance());
        swerve.normalDriveRequest();
      }
      case FORCE_SCORE -> {
        vision.setState(VisionState.TAGS);
        shooter.scoreRequest(scoringParameters.distance());
        shooterHood.scoreRequest(scoringParameters.distance());
        swerve.normalDriveRequest();
      }
      case PREPARE_FEED -> {
        vision.setState(VisionState.TAGS);
        shooter.feedRequest(feedingParameters.distance());
        shooterHood.feedRequest(feedingParameters.distance());
        // Deploy is controlled separately
        // Intake is controlled separately
        swerve.rateLimitedDriveRequest();
      }
      case FEED -> {
        vision.setState(VisionState.TAGS);
        shooter.feedRequest(feedingParameters.distance());
        shooterHood.feedRequest(feedingParameters.distance());
        swerve.rateLimitedDriveRequest();
      }
      case STOP_SHOOTING_FEED -> {
        vision.setState(VisionState.TAGS);
        shooter.feedRequest(feedingParameters.distance());
        shooterHood.feedRequest(feedingParameters.distance());
        swerve.normalDriveRequest();
      }
      case PREPARE_SCORE -> {
        vision.setState(VisionState.HUB_TAGS);
        shooter.scoreRequest(scoringParameters.distance());
        // Deploy is controlled separately
        // Intake is controlled separately
        swerve.rateLimitedDriveRequest();
      }
      case SCORE -> {
        vision.setState(VisionState.HUB_TAGS);
        shooter.scoreRequest(scoringParameters.distance());
        shooterHood.scoreRequest(scoringParameters.distance());
        swerve.rateLimitedDriveRequest();
      }
      case STOP_SHOOTING_SCORE -> {
        vision.setState(VisionState.HUB_TAGS);
        shooter.scoreRequest(scoringParameters.distance());
        shooterHood.scoreRequest(scoringParameters.distance());
        swerve.rateLimitedDriveRequest();
      }
      case PREPARE_PRESET_FEED -> {
        vision.setState(VisionState.TAGS);
        shooter.feedRequest(PRESET_FEED_DISTANCE);
        shooterHood.feedRequest(PRESET_FEED_DISTANCE);
        // Deploy is controlled separately
        // Intake is controlled separately
        swerve.normalDriveRequest();
      }
      case PRESET_FEED -> {
        vision.setState(VisionState.TAGS);
        shooter.feedRequest(PRESET_FEED_DISTANCE);
        shooterHood.feedRequest(PRESET_FEED_DISTANCE);
        swerve.normalDriveRequest();
      }
      case STOP_SHOOTING_PRESET_FEED -> {
        vision.setState(VisionState.TAGS);
        shooter.feedRequest(PRESET_FEED_DISTANCE);
        shooterHood.feedRequest(PRESET_FEED_DISTANCE);
        swerve.normalDriveRequest();
      }
      case PREPARE_PRESET_SCORE -> {
        vision.setState(VisionState.TAGS);
        shooter.scoreRequest(scoringParameters.distance());
        shooterHood.scoreRequest(scoringParameters.distance());
        swerve.normalDriveRequest();
      }
      case PRESET_SCORE -> {
        vision.setState(VisionState.TAGS);
        shooter.scoreRequest(scoringParameters.distance());
        shooterHood.scoreRequest(scoringParameters.distance());
        swerve.normalDriveRequest();
      }
      case STOP_SHOOTING_PRESET_SCORE -> {
        vision.setState(VisionState.TAGS);
        shooter.scoreRequest(scoringParameters.distance());
        shooterHood.scoreRequest(scoringParameters.distance());
        swerve.normalDriveRequest();
      }
      case UNJAM -> {
        vision.setState(VisionState.TAGS);
        shooter.idleRequest();
        shooterHood.idleRequest();
        // Deploy is controlled separately
        swerve.normalDriveRequest();
      }
    }
  }

  @Override
  protected void whileInState(RobotState state) {
    switch (state) {
      case IDLE, UNJAM -> {
        smartHoodIdleRequest();
        if (hopperManager.getState().isIntaking()) {
          swerve.intakeDriveRequest();
        } else {
          swerve.normalDriveRequest();
        }
      }

      case PREPARE_SCORE, STOP_SHOOTING_SCORE -> {
        smartHoodPrepareScoreRequest();
        if (hopperManager.getState().isIntaking()) {
          swerve.intakeRateLimitedDriveRequest();
        } else {
          swerve.rateLimitedDriveRequest();
        }
        // isHubActive always logged
      }
      case SCORE -> {
        shooterHood.scoreRequest(scoringParameters.distance());
        if (hopperManager.getState().isIntaking()) {
          swerve.intakeRateLimitedDriveRequest();
        } else {
          swerve.rateLimitedDriveRequest();
        }
        // TODO:Implement hoppermanager shuffle here
      }
      case PREPARE_FEED -> {
        smartHoodPrepareFeedRequest();
        if (hopperManager.getState().isIntaking()) {
          swerve.intakeRateLimitedDriveRequest();
        } else {
          swerve.rateLimitedDriveRequest();
        }
      }
      case FEED -> {
        shooterHood.feedRequest(feedingParameters.distance());
        if (hopperManager.getState().isIntaking()) {
          swerve.intakeRateLimitedDriveRequest();
        } else {
          swerve.rateLimitedDriveRequest();
        }
        // TODO: implement hoppermanager shuffle here
      }

      // Fallback states
      case PREPARE_PRESET_SCORE -> {
        // Automatically update scoring parameters with preset pose
        if (isMoving) {
          shooterHood.idleRequest();
        } else {
          shooterHood.scoreRequest(scoringParameters.distance());
        }
        if (hopperManager.getState().isIntaking()) {
          swerve.intakeDriveRequest();
        } else {
          swerve.normalDriveRequest();
        }
      }
      case PRESET_SCORE -> {
        // Automatically update scoring parameters with preset pose
        shooterHood.scoreRequest(scoringParameters.distance());
        if (hopperManager.getState().isIntaking()) {
          swerve.intakeDriveRequest();
        } else {
          swerve.normalDriveRequest();
        }
        // TODO: Shuffle

      }
      case PREPARE_PRESET_FEED -> {
        if (hopperManager.getState().isIntaking()) {
          swerve.intakeDriveRequest();
        } else {
          swerve.normalDriveRequest();
        }
      }
      case PRESET_FEED -> {
        if (hopperManager.getState().isIntaking()) {
          swerve.intakeDriveRequest();
        } else {
          swerve.normalDriveRequest();
        }
        // TODO: Shuffle

      }
      default -> {}
    }
    DogLog.log("RobotManager/Feeding/FeedLocation", feedLocation);
    DogLog.log("RobotManager/Feeding/FeedParameters", feedingParameters);
    DogLog.log("RobotManager/Scoring/ScoringParameters", scoringParameters);

    DogLog.log("RobotManager/DrivingToIntake", drivingToIntake);

    DogLog.log("RobotManager/gpDetection/Shooter", shooter.currentDetectsGp());
    DogLog.log("RobotManager/gpDetection/both", detectingGp());

    MechanismVisualizer.log(robotPose, shooterHood.getAngle(), hopperManager.deploy.getPosition());
  }

  private void smartHoodIdleRequest() {
    // -First, if cameras are offline or we are near a trench, always be idle
    // -Otherwise if we are in our alliance zone, point towards hub
    // -And if we are not in alliance zone, point towards feed pose
    if (!health.isLocalizationHealthy() || !localization.isTrustworthy() || nearTrench) {
      shooterHood.idleRequest();

      DogLog.log("RobotManager/SmartIdle/Status", "NearTrench");
    } else if (FieldUtil.isRobotPastObstacleTowardAllianceZone(robotPose.getTranslation())) {
      shooterHood.idleRequest();

      DogLog.log("RobotManager/SmartIdle/Status", "InAllianceZone");
    } else if (DriverStation.isAutonomous()) {
      shooterHood.idleRequest();
      DogLog.log("RobotManager/SmartIdle/Status", "UseHubForAuto");
    } else {
      shooterHood.idleRequest();
      DogLog.log("RobotManager/SmartIdle/Status", "NotInAlliance");
    }
  }

  private void smartHoodPrepareScoreRequest() {
    if (!health.isLocalizationHealthy() || nearTrench) {
      shooterHood.idleRequest();
      DogLog.log("RobotManager/Scoring/SmartPrepareScore/HoodStatus", "NearTrench");
    } else {
      DogLog.log("RobotManager/Scoring/SmartPrepareScore/HoodStatus", "NotNearTrench");

      shooterHood.scoreRequest(scoringParameters.distance());
    }
  }

  private void smartHoodPrepareFeedRequest() {
    if (!health.isLocalizationHealthy() || nearTrench) {
      shooterHood.idleRequest();
      DogLog.log("RobotManager/Scoring/SmartPrepareScore/HoodStatus", "NearTrench");
    } else {
      DogLog.log("RobotManager/Scoring/SmartPrepareScore/HoodStatus", "NotNearTrench");

      shooterHood.feedRequest(feedingParameters.distance());
    }
  }

  public void idleRequest() {
    switch (getState()) {
      case SCORE -> setStateFromRequest(RobotState.STOP_SHOOTING_SCORE);
      case PRESET_SCORE -> setStateFromRequest(RobotState.STOP_SHOOTING_PRESET_SCORE);
      case FEED -> setStateFromRequest(RobotState.STOP_SHOOTING_FEED);
      case PRESET_FEED -> setStateFromRequest(RobotState.STOP_SHOOTING_PRESET_FEED);
      default -> setStateFromRequest(RobotState.IDLE);
    }
  }

  public void forceShootRequest() {
    if (getState() != RobotState.FORCE_SCORE) {
      setStateFromRequest(RobotState.PREPARE_FORCE_SCORE);
    }
  }

  public void prepareScoreRequest() {
    if (getState() != RobotState.PRESET_SCORE && getState() != RobotState.SCORE) {
      if (!health.isLocalizationHealthy()) {
        setStateFromRequest(RobotState.PREPARE_PRESET_SCORE);
      } else {
        setStateFromRequest(RobotState.PREPARE_SCORE);
      }
    }
  }

  public void prepareFeedRequest() {
    if (getState() != RobotState.PRESET_FEED && getState() != RobotState.FEED) {
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
        // TODO: finish figure out shoot and score logic
        case SCORE, FEED, FORCE_SCORE, PRESET_SCORE, PRESET_FEED -> hopperManager.scoreRequest();
        default -> hopperManager.intakeRequest();
      }
    } else {
      switch (getState()) {
        case SCORE, FEED, FORCE_SCORE, PRESET_SCORE, PRESET_FEED -> hopperManager.scoreRequest();
        default -> hopperManager.idleRequest();
      }
    }
  }

  public void stowDeployRequest() {
    hopperManager.deploy.stowRequest();
    if (driverWantsToIntake) {
      hopperManager.intakeRequest();
    } else {
      hopperManager.idleRequest();
    }
  }

  public void intakeAutoRequest() {
    hopperManager.intakeRequest();
  }

  public void cancelIntakeRequest() {
    hopperManager.idleRequest();

    // TODO: Figure out shuffle logic
    // If we are shooting while cancelling a previous intake request, send a new
    // hopper shuffle request
    // switch (getState()) {
    //   case PREPARE_FORCE_SCORE,
    //       FORCE_SCORE,
    //       PREPARE_PRESET_SCORE,
    //       PRESET_SCORE,
    //       PREPARE_SCORE,
    //       SCORE,
    //       PREPARE_PRESET_FEED,
    //       PRESET_FEED,
    //       PREPARE_FEED,
    //       FEED ->
    //       deploy.shuffleRequest();
    //   default -> {}
    // }
  }

  public boolean detectingGp() {
    return shooter.currentDetectsGp();
  }

  public void unjamRequest() {
    setStateFromRequest(RobotState.UNJAM);
  }

  public void homeDeployRequest() {
    hopperManager.rehomeDeployRequest();
  }

  public void homeDeployInAutoRequest() {
    hopperManager.rehomeDeployRequest();
  }

  public void homeShooterHoodRequest() {
    shooterHood.homingRequest();
  }

  @Override
  protected void collectInputs() {
    hubActivity.updateShooterScoringTOF(shooter.getScoreTimeOfFlight(scoringParameters.distance()));

    robotPose = localization.getPose();
    double robotRotation = robotPose.getRotation().getDegrees();
    clusterMap.setDeployFullyExtended(hopperManager.deploy.isFullyExtended());
    vision.setEstimatedPoseAngle(robotRotation);

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

    feedingParameters =
        AimParameterUtil.getFeedingParameters(
            feedLocation, robotPose, swerve.getFieldRelativeSpeeds());

    fallbackFeedingParameters =
        AimParameterUtil.getFallbackFeedingParameters(robotPose.getRotation());

    var swerveVector = MathHelpers.getDriveDirection(speeds);
    double driveDirection = swerveVector.getDegrees();
    drivingToIntake =
        hopperManager.getState().isIntaking()
            && MathUtil.isNear(robotRotation, driveDirection, 120.0, -180, 180)
            && MathHelpers.getLinearVelocity(speeds) > 1e-5;
    isInAllianceZone =
        !health.isLocalizationHealthy()
            ? hubActivity.getTOFBasedHubActive()
            : FieldUtil.isRobotPastObstacleTowardAllianceZone(robotPose.getTranslation());

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
    DogLog.log("RobotManager/Feeding/FeedTransition/ShooterHoodAtGoal", shooterHood.atGoal());
    DogLog.log(
        "RobotManager/Feeding/FeedTransition/LocalizationHealthy", health.isLocalizationHealthy());
    DogLog.log("RobotManager/Feeding/FeedTransition/ImuFlat", localization.imu.isFlatDebounced());
    DogLog.log("RobotManager/Feeding/ScoreTransition/NotNearTrench", !nearTrench);
  }
}
