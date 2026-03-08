package frc.robot.robot_manager;

import com.team581.util.AprilTags;
import com.team581.util.FieldUtil;
import com.team581.util.FmsUtil;
import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.localization.Localization;
import frc.robot.swerve.Swerve;
import frc.robot.turret.Turret;
import frc.robot.turret.TurretCalculator;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.vision.Vision;
import frc.robot.vision.VisionState;
import java.util.Optional;

public class RobotManager extends StateMachineSubsystem<RobotState> {
  private static final IntegerSubscriber TAG_AIM_ID = DogLog.tunable("TagAimID", 9);
  private static final DoubleSubscriber TIME_OF_FLIGHT = DogLog.tunable("TimeOfFlight", 0.0);

  public final Localization localization;
  public final Swerve swerve;
  public final Turret turret;
  public final Vision vision;
  private Optional<RobotState> beforeAutoScoreState = Optional.empty();

  private Pose2d robotPose = Pose2d.kZero;

  /** The robot translation clamped to be within the alliance zone. */
  private Pose2d robotPoseInAllianceZone = Pose2d.kZero;

  private double startOfMatchPeriod = 0;
  private double timeSinceMatchStart = 0;

  private boolean autoscore = false;
  private boolean inAllianceZone = true;
  private boolean readyToShootAtHub = true;
  private double swerveTurretCompensationAngle = 0.0;
  private double turretHubGoalAngle = 0.0;

  public RobotManager(Localization localization, Swerve swerve, Turret turret, Vision vision) {
    super(SubsystemPriority.ROBOT_MANAGER, RobotState.IDLE);
    this.turret = turret;
    this.localization = localization;
    this.swerve = swerve;
    this.vision = vision;
  }

  @Override
  protected RobotState getNextState(RobotState currentState) {
    // TODO: this needs to check if we have balls in the future
    if (autoscore) {
      if (readyToShootAtHub && inAllianceZone) {
        if (turret.goalOutOfBounds()) {
          swerveTurretCompensationAngle =
              TurretCalculator.calculateSwerveTurretCompensationAngle(
                  turretHubGoalAngle, robotPose.getRotation());
          return RobotState.HUB_AIM_ADJUSTING_SWERVE;
        }
        return RobotState.HUB_AIM;
      }
    }
    return switch (currentState) {
      case HUB_AIM -> {
        if (turret.goalOutOfBounds()) {
          swerveTurretCompensationAngle =
              TurretCalculator.calculateSwerveTurretCompensationAngle(
                  turretHubGoalAngle, robotPose.getRotation());
          yield RobotState.HUB_AIM_ADJUSTING_SWERVE;
        }
        yield currentState;
      }
      case HUB_AIM_ADJUSTING_SWERVE -> {
        if (MathUtil.isNear(
                swerveTurretCompensationAngle, robotPose.getRotation().getDegrees(), 5, -180, 180)
            || TurretCalculator.doesTurretHaveRoom(turret.getAngle())) {
          yield RobotState.HUB_AIM;
        }
        yield currentState;
      }
      default -> currentState;
    };
  }

  @Override
  public void autonomousInit() {
    startOfMatchPeriod = Timer.getFPGATimestamp();
    timeSinceMatchStart = 0.0;
  }

  @Override
  public void teleopInit() {
    startOfMatchPeriod = Timer.getFPGATimestamp() - 20.0;
    timeSinceMatchStart = 20.0;
  }

  @Override
  protected void afterTransition(RobotState newState) {
    switch (newState) {
      case HUB_AIM -> {
        vision.setState(VisionState.HUB_TAGS);
        turret.hubAimRequest();
        swerve.normalDriveRequest();
      }
      case HUB_AIM_ADJUSTING_SWERVE -> {
        vision.setState(VisionState.HUB_TAGS);
        turret.hubAimRequest();
      }
      case TAG_AIM -> {
        vision.setState(VisionState.TAGS);
        turret.tagAimRequest();
      }
      case LOCK_FORWARD -> {
        turret.lockForwardRequest();
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
      }
      case IDLE -> {
        vision.setState(VisionState.TAGS);
        turret.idleRequest();
      }
    }
  }

  @Override
  protected void whileInState(RobotState state) {
    DogLog.log("RobotManager/SwerveCompAngle", swerveTurretCompensationAngle);

    switch (getState()) {
      case HUB_AIM -> {
        turret.setHubAimAngle(turretHubGoalAngle);
      }
      case HUB_AIM_ADJUSTING_SWERVE -> {
        swerve.hubAimRequest(swerveTurretCompensationAngle);
        var goalPose = FieldUtil.HUB_POSE.getTranslation();
        // Disabled since new shoot on the move implementation has a different interface
        // var goalPose =
        //     ShootOnTheMove.getVelocityCompensatedGoal(
        //         FieldUtil.HUB_POSE.getTranslation(),
        //         swerve.getFieldRelativeSpeeds(),
        //         TIME_OF_FLIGHT.get());
        var aimingAngle =
            TurretCalculator.calculateTurretAimingAngle(robotPoseInAllianceZone, goalPose);
        turret.setHubAimAngle(aimingAngle);
      }
      case TAG_AIM -> {
        var goalPose = AprilTags.getTagPose((int) TAG_AIM_ID.get());
        if (goalPose != null) {
          var aimingAngle =
              TurretCalculator.calculateTurretAimingAngle(
                  robotPoseInAllianceZone, goalPose.getTranslation());
          turret.setTagAimAngle(aimingAngle);
          DogLog.clearFault("Tag Aim: No valid ID");
        } else {
          DogLog.logFault("Tag Aim: No valid ID");
          setStateFromRequest(RobotState.IDLE);
        }
      }
      default -> {
        swerve.intakeDriveRequest();
      }
    }

    MechanismVisualizer.log(localization.getPose(), turret.getAngle());
  }

  @Override
  protected void collectInputs() {
    robotPose = localization.getPose();
    var robotTranslation = robotPose.getTranslation();

    timeSinceMatchStart = Timer.getFPGATimestamp() - startOfMatchPeriod;

    var robotVelocity = Math.toDegrees(swerve.getFieldRelativeSpeeds().omegaRadiansPerSecond);
    DogLog.log("RobotManager/RobotVelocity", robotVelocity);
    var turretVelocity = turret.getVelocityDegreesPerSecond();
    DogLog.log("RobotManager/TurretVelocity", turretVelocity);

    readyToShootAtHub = FmsUtil.isHubActive(timeSinceMatchStart + TIME_OF_FLIGHT.get(), true);
    inAllianceZone = FieldUtil.isRobotInAllianceZone(robotTranslation);
    robotPoseInAllianceZone = FieldUtil.clampPoseToAllianceZone(robotPose);
    DogLog.log("RobotManager/Autoscore", autoscore);
    DogLog.log("RobotManager/ReadyToShootAtHub", readyToShootAtHub);
    DogLog.log("RobotManager/InAllianceZone", inAllianceZone);

    var goalPose = FieldUtil.HUB_POSE.getTranslation();

    turret.setDistance(robotTranslation.getDistance(goalPose));
    // Disabled since new shoot on the move implementation has a different interface
    // if (FeatureFlags.SHOOT_ON_THE_MOVE.getAsBoolean()) {
    //   goalPose =
    //       ShootOnTheMove.getVelocityCompensatedGoal(
    //           goalPose, swerve.getFieldRelativeSpeeds(), TIME_OF_FLIGHT.get());
    // }

    turretHubGoalAngle =
        TurretCalculator.calculateTurretAimingAngle(robotPoseInAllianceZone, goalPose);

    swerve.setVisionOnline(vision.isAnyCameraOnlineForTags());
  }

  public void hubAimRequest() {
    if (getState() != RobotState.HUB_AIM_ADJUSTING_SWERVE) {
      setStateFromRequest(RobotState.HUB_AIM);
    }
  }

  public void setAutoscoreRequest(boolean autoScoring) {
    if (autoscore && !autoScoring) {
      setStateFromRequest(beforeAutoScoreState.orElse(RobotState.IDLE));
      beforeAutoScoreState = Optional.empty();
    }
    if (!autoscore && autoScoring) {
      beforeAutoScoreState = Optional.of(getState());
    }
    autoscore = autoScoring;
  }

  public void toggleAutoscoreRequest() {
    setAutoscoreRequest(!autoscore);
  }

  public void tagAimRequest() {
    setStateFromRequest(RobotState.TAG_AIM);
  }

  public void lockForwardRequest() {
    setStateFromRequest(RobotState.LOCK_FORWARD);
  }
}
