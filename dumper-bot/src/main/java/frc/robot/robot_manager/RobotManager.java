package frc.robot.robot_manager;

import com.team581.math.ShootOnTheMove;
import com.team581.util.FieldUtil;
import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.config.FeatureFlags;
import frc.robot.feeder.Feeder;
import frc.robot.hopper.Hopper;
import frc.robot.intake.Intake;
import frc.robot.localization.Localization;
import frc.robot.shooter.Shooter;
import frc.robot.swerve.Swerve;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.vision.Vision;
import frc.robot.vision.VisionState;

public class RobotManager extends StateMachineSubsystem<RobotState> {
  private static Translation2d HUB_GOAL_POSE = FieldUtil.getHubPose();
  private static Translation2d FEED_1_GOAL_POSE = FieldUtil.getFeed1Pose();
  private static Translation2d FEED_2_GOAL_POSE = FieldUtil.getFeed2Pose();

  private final Intake intake;
  private final Hopper hopper;
  private final Shooter shooter;
  private final Feeder feeder;
  private final Swerve swerve;
  private final Vision vision;
  private final Localization localization;

  private Pose2d robotPose = Pose2d.kZero;
  private double hubGoalAngle = 0.0;
  private double feed1GoalAngle = 0.0;
  private double feed2GoalAngle = 0.0;
  private double hubDistance = 0.0;
  private double feed1Distance = 0.0;
  private double feed2Distance = 0.0;
  private double timeOfFlight = 0.0;

  public RobotManager(
      Intake intake,
      Hopper hopper,
      Shooter shooter,
      Feeder feeder,
      Swerve swerve,
      Vision vision,
      Localization localization) {
    super(SubsystemPriority.ROBOT_MANAGER, RobotState.IDLE);
    this.intake = intake;
    this.hopper = hopper;
    this.shooter = shooter;
    this.feeder = feeder;
    this.swerve = swerve;
    this.vision = vision;
    this.localization = localization;

    DogLog.log("Robot/StateCount", RobotState.values().length);
  }

  @Override
  protected RobotState getNextState(RobotState currentState) {
    return switch (currentState) {
      // case PREPARE_SHOOT_HUB -> {
      //   if (shooter.atGoal() && vision.seeingTag() && FieldUtil.isRobotInAllianceZone(robotPose))
      // {
      //     yield RobotState.SHOOT_HUB;
      //   }
      //   yield currentState;
      // }
      case PREPARE_SHOOT_HUB -> {
        if (shooter.atGoal()) {
          yield RobotState.SHOOT_HUB;
        }
        yield currentState;
      }
      case PREPARE_FORCE_SHOOT -> {
        if (shooter.atGoal()) {
          yield RobotState.FORCE_SHOOT;
        }
        yield currentState;
      }
      case PREPARE_FEED_1 -> shooter.atGoal() ? RobotState.FEED_1 : currentState;
      case PREPARE_FEED_2 -> shooter.atGoal() ? RobotState.FEED_2 : currentState;
      case SHOOT_HUB ->
          !FieldUtil.isRobotInAllianceZone(robotPose) ? RobotState.IDLE : currentState;
      default -> currentState;
    };
  }

  @Override
  protected void afterTransition(RobotState newState) {
    switch (newState) {
      case IDLE -> {
        vision.setState(VisionState.TAGS);

        shooter.idleRequest();
        feeder.idleRequest();
        intake.idleRequest();
        hopper.idleRequest();
        swerve.normalDriveRequest();
      }
      case PREPARE_FORCE_SHOOT -> {
        vision.setState(VisionState.HUB_TAGS);

        shooter.scoreRequest(hubDistance);
        feeder.idleRequest();
        // Intake is controlled separately
        hopper.idleRequest();
        swerve.normalDriveRequest();
      }
      case FORCE_SHOOT -> {
        vision.setState(VisionState.HUB_TAGS);

        shooter.scoreRequest(hubDistance);
        feeder.feedRequest();
        intake.shootingRequest();
        hopper.shootRequest();
        swerve.normalDriveRequest();
      }
      case WAIT_FEED_1, PREPARE_FEED_1 -> {
        vision.setState(VisionState.TAGS);

        shooter.feedRequest(feed1Distance);
        feeder.idleRequest();
        // Intake is controlled separately
        hopper.idleRequest();
        swerve.snapsDriveRequest(feed1GoalAngle);
      }
      case WAIT_FEED_2, PREPARE_FEED_2 -> {
        vision.setState(VisionState.TAGS);

        shooter.feedRequest(feed2Distance);
        feeder.idleRequest();
        // Intake is controlled separately
        hopper.idleRequest();
        swerve.snapsDriveRequest(feed2GoalAngle);
      }
      case FEED_1 -> {
        vision.setState(VisionState.TAGS);

        shooter.feedRequest(feed1Distance);
        feeder.feedRequest();
        intake.shootingRequest();
        hopper.shootRequest();
        swerve.snapsDriveRequest(feed1GoalAngle);
      }
      case FEED_2 -> {
        vision.setState(VisionState.TAGS);

        shooter.feedRequest(feed2Distance);
        feeder.feedRequest();
        intake.shootingRequest();
        hopper.shootRequest();
        swerve.snapsDriveRequest(feed2GoalAngle);
      }
      case WAIT_SHOOT_HUB, PREPARE_SHOOT_HUB -> {
        vision.setState(VisionState.HUB_TAGS);

        shooter.scoreRequest(hubDistance);
        feeder.idleRequest();
        // Intake is controlled separately
        hopper.idleRequest();
        swerve.snapsDriveRequest(hubGoalAngle);
      }
      case SHOOT_HUB -> {
        vision.setState(VisionState.HUB_TAGS);
        shooter.scoreRequest(hubDistance);
        feeder.feedRequest();
        intake.shootingRequest();
        hopper.shootRequest();
        swerve.snapsDriveRequest(hubGoalAngle);
      }
      default -> {}
    }
  }

  @Override
  protected void whileInState(RobotState state) {
    switch (state) {
      case WAIT_FEED_1, PREPARE_FEED_1, FEED_1 -> swerve.snapsDriveRequest(feed1GoalAngle);
      case WAIT_FEED_2, PREPARE_FEED_2, FEED_2 -> swerve.snapsDriveRequest(feed2GoalAngle);
      case WAIT_SHOOT_HUB, PREPARE_SHOOT_HUB, SHOOT_HUB -> swerve.snapsDriveRequest(hubGoalAngle);
      default -> swerve.normalDriveRequest();
    }

    DogLog.log("RobotManager/HubDistance", hubDistance);
  }

  @Override
  protected void collectInputs() {
    robotPose = localization.getPose();
    timeOfFlight = shooter.getCurrentTimeOfFlight();

    if (FeatureFlags.SHOOT_ON_THE_MOVE.getAsBoolean()) {
      HUB_GOAL_POSE =
          ShootOnTheMove.getVelocityCompensatedGoal(
              FieldUtil.getHubPose(), swerve.getFieldRelativeSpeeds(), timeOfFlight);
      FEED_1_GOAL_POSE =
          ShootOnTheMove.getVelocityCompensatedGoal(
              FieldUtil.getFeed1Pose(), swerve.getFieldRelativeSpeeds(), timeOfFlight);
      FEED_2_GOAL_POSE =
          ShootOnTheMove.getVelocityCompensatedGoal(
              FieldUtil.getFeed2Pose(), swerve.getFieldRelativeSpeeds(), timeOfFlight);
    } else {
      HUB_GOAL_POSE = FieldUtil.getHubPose();
      FEED_1_GOAL_POSE = FieldUtil.getFeed1Pose();
      FEED_2_GOAL_POSE = FieldUtil.getFeed2Pose();
    }

    hubGoalAngle = getSwerveAimingAngle(HUB_GOAL_POSE);
    feed1GoalAngle = getSwerveAimingAngle(FEED_1_GOAL_POSE);
    feed2GoalAngle = getSwerveAimingAngle(FEED_2_GOAL_POSE);

    hubDistance = robotPose.getTranslation().getDistance(HUB_GOAL_POSE);
    feed1Distance = robotPose.getTranslation().getDistance(FEED_1_GOAL_POSE);
    feed2Distance = robotPose.getTranslation().getDistance(FEED_2_GOAL_POSE);
  }

  private double getSwerveAimingAngle(Translation2d goal) {
    return Units.radiansToDegrees(
        Math.atan2(goal.getY() - robotPose.getY(), goal.getX() - robotPose.getX()));
  }

  private void setStateFailSafe(RobotState newState) {
    if (getState().climbingOrRehoming()) {
      return;
    }
    setStateFromRequest(newState);
  }

  public void idleRequest() {
    setStateFailSafe(RobotState.IDLE);
  }

  public void forceShootRequest() {
    setStateFailSafe(RobotState.PREPARE_FORCE_SHOOT);
  }

  public void shootHubWaitRequest() {
    setStateFailSafe(RobotState.WAIT_SHOOT_HUB);
  }

  public void feed1WaitRequest() {
    setStateFailSafe(RobotState.WAIT_FEED_1);
  }

  public void feed2WaitRequest() {
    setStateFailSafe(RobotState.WAIT_FEED_2);
  }

  public void toggleHubRequest() {
    switch (getState()) {
      case PREPARE_SHOOT_HUB, SHOOT_HUB -> setStateFailSafe(RobotState.IDLE);
      default -> setStateFailSafe(RobotState.PREPARE_SHOOT_HUB);
    }
  }

  public void intakeRequest() {
    intake.intakeRequest();
    hopper.intakeRequest();
  }

  public void cancelIntakeRequest() {
    intake.idleRequest();
    hopper.idleRequest();
  }

  public void toggleFeedRequest() {
    switch (getState()) {
      case WAIT_FEED_2 -> setStateFailSafe(RobotState.PREPARE_FEED_2);
      case PREPARE_FEED_1, PREPARE_FEED_2, FEED_1, FEED_2 -> setStateFailSafe(RobotState.IDLE);
      default -> setStateFailSafe(RobotState.PREPARE_FEED_1);
    }
  }

  public void climbSequenceForward() {
    intake.idleRequest();
    switch (getState()) {
      default -> setStateFromRequest(RobotState.CLIMB_1_LINEUP_L1);
      case CLIMB_1_LINEUP_L1 -> setStateFromRequest(RobotState.CLIMB_2_RAISING_L1);
      case CLIMB_2_RAISING_L1 -> setStateFromRequest(RobotState.CLIMB_3_HANGING_L1);
      case CLIMB_3_HANGING_L1 -> setStateFromRequest(RobotState.CLIMB_4_RAISING_L2);

      case CLIMB_4_RAISING_L2 -> setStateFromRequest(RobotState.CLIMB_5_HANGING_L2);
      case CLIMB_5_HANGING_L2 -> setStateFromRequest(RobotState.CLIMB_6_RAISING_L3);

      case CLIMB_6_RAISING_L3 -> setStateFromRequest(RobotState.CLIMB_7_HANGING_L3);
      case CLIMB_7_HANGING_L3 -> {}
    }
  }

  public void climbSequenceForwardAuto() {
    intake.idleRequest();
    switch (getState()) {
      default -> setStateFromRequest(RobotState.CLIMB_1_LINEUP_L1_AUTO);
      case CLIMB_1_LINEUP_L1_AUTO -> setStateFromRequest(RobotState.CLIMB_2_RAISING_L1_AUTO);
      case CLIMB_2_RAISING_L1_AUTO -> setStateFromRequest(RobotState.CLIMB_3_HANGING_L1_AUTO);
      case CLIMB_3_HANGING_L1_AUTO -> {}
    }
  }

  public void climbSequenceBackward() {
    intake.idleRequest();
    switch (getState()) {
      default -> {}
      case CLIMB_1_LINEUP_L1 -> setStateFromRequest(RobotState.IDLE);
      case CLIMB_2_RAISING_L1 -> setStateFromRequest(RobotState.CLIMB_1_LINEUP_L1);
      case CLIMB_3_HANGING_L1 -> setStateFromRequest(RobotState.CLIMB_2_RAISING_L1);

      case CLIMB_1_LINEUP_L1_AUTO -> setStateFromRequest(RobotState.IDLE);
      case CLIMB_2_RAISING_L1_AUTO -> setStateFromRequest(RobotState.CLIMB_1_LINEUP_L1_AUTO);
      case CLIMB_3_HANGING_L1_AUTO -> setStateFromRequest(RobotState.CLIMB_2_RAISING_L1_AUTO);

      case CLIMB_4_RAISING_L2 -> setStateFromRequest(RobotState.CLIMB_3_HANGING_L1);
      case CLIMB_5_HANGING_L2 -> setStateFromRequest(RobotState.CLIMB_4_RAISING_L2);

      case CLIMB_6_RAISING_L3 -> setStateFromRequest(RobotState.CLIMB_5_HANGING_L2);
      case CLIMB_7_HANGING_L3 -> setStateFromRequest(RobotState.CLIMB_6_RAISING_L3);
    }
  }
}
