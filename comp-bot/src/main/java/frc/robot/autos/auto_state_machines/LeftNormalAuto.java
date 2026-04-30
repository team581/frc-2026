package frc.robot.autos.auto_state_machines;

import com.team581.autos.Point;
import com.team581.autos.StuckOnBallRecovery;
import com.team581.math.PoseErrorTolerance;
import com.team581.mechanisms.imu.BumpCrossingState;
import com.team581.mechanisms.imu.BumpCrossingTracker;
import com.team581.trailblazer.AutoPoint;
import com.team581.trailblazer.Trailblazer;
import com.team581.trailblazer.segments.AutoSegment;
import com.team581.util.FieldUtil;
import com.team581.util.FmsUtil;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.autos.BaseImperativeAuto;
import frc.robot.autos.auto_state_machines.auto_state.NormalAutoState;
import frc.robot.cluster_map.Lane;
import frc.robot.config.FeatureFlags;
import frc.robot.robot_manager.RobotManager;

public class LeftNormalAuto extends BaseImperativeAuto<NormalAutoState> {

  private static final double SHOOTER_WARMUP_DISTANCE_FROM_HUB = 2.171;

  public enum Markers {
    PRIORITIZE_INTAKE,
    START_SHOOT_RQ,
    READY_TO_SHOOT_FOR_2,
    START_INTAKE_2,
    START_INTAKE_3,
    MAKE_CLUSTER_MAP_DECISION,
    CANCEL_CLUSTER_MAP_CHECK,
    READY_TO_CROSS_BUMP
  }

  private BumpCrossingTracker bumpCrossingTracker;

  private static final double COLLISION_X_OFFSET = 0.13;
  private static final double MAX_CLUSTER_MAP_OFFSET = 0.35;

  private static final double MIDLINE_OFFSET = 0.0;

  private static final double BUMP_OFFSET = -0.13;

  private static final double SHOOT_X = 13.83;

  private boolean collisionEverDetected = false;

  private final AutoSegment intakeFirstCycle =
      Trailblazer.segment(
              AutoPoint.ofRed(
                      new Pose2d(
                          10.489, FieldUtil.RED_DEPOT_TRENCH_CENTER.getY(), Rotation2d.kCCW_90deg))
                  .withTransitionTolerance(new PoseErrorTolerance(0.75, 100)),
              AutoPoint.of(
                      () ->
                          getCollisionPoint(
                              Point.ofRed(new Pose2d(9.010, 1.417, Rotation2d.fromDegrees(120)))))
                  .withTransitionTolerance(new PoseErrorTolerance(0.4, 100)),
              AutoPoint.of(
                      () ->
                          getCollisionPoint(
                              Point.ofRed(new Pose2d(8.553, 2.677, Rotation2d.fromDegrees(85)))))
                  .withTransitionTolerance(new PoseErrorTolerance(0.4, 100)),
              AutoPoint.of(
                      () ->
                          getCollisionPoint(
                              Point.ofRed(new Pose2d(8.670, 3.37, Rotation2d.fromDegrees(30)))))
                  .withMarker(Markers.PRIORITIZE_INTAKE)
                  .withTransitionTolerance(new PoseErrorTolerance(0.4, 100)),
              AutoPoint.ofRed(new Pose2d(9.6, 3.57, Rotation2d.fromDegrees(-90)))
                  .withTransitionTolerance(new PoseErrorTolerance(0.4, 100)),
              AutoPoint.ofRed(
                      new Pose2d(
                          9.7,
                          FieldUtil.RED_DEPOT_BUMP_CENTER.getY() - BUMP_OFFSET,
                          Rotation2d.fromDegrees(-32)))
                  .withTransitionTolerance(new PoseErrorTolerance(0.3, 100)),
              AutoPoint.ofRed(
                      new Pose2d(
                          10.6,
                          FieldUtil.RED_DEPOT_BUMP_CENTER.getY() - BUMP_OFFSET,
                          Rotation2d.kZero))
                  .withTransitionTolerance(new PoseErrorTolerance(0.3, 100)),
              AutoPoint.ofRed(
                      new Pose2d(
                          SHOOT_X,
                          FieldUtil.RED_DEPOT_BUMP_CENTER.getY() - BUMP_OFFSET,
                          Rotation2d.kZero))
                  .withMarker(Markers.READY_TO_CROSS_BUMP)
                  .withTransitionTolerance(new PoseErrorTolerance(0.3, 100)))
          .withLinearConstraints(4.5, 8)
          .withAngularConstraints(Units.rotationsToRadians(4.0), Units.rotationsToRadians(4.0))
          .untilFinished(new PoseErrorTolerance(0.2, 3));

  private final AutoSegment crossBumpToShootOne =
      Trailblazer.segment(
              AutoPoint.ofRed(
                      new Pose2d(
                          SHOOT_X,
                          FieldUtil.RED_DEPOT_BUMP_CENTER.getY() - BUMP_OFFSET,
                          Rotation2d.kZero))
                  .withTransitionTolerance(new PoseErrorTolerance(0.1, 100))
                  .withLinearConstraints(4.5, 8)
                  .withMarker(Markers.START_SHOOT_RQ))
          .withLinearConstraints(4.5, 8)
          .withAngularConstraints(Units.rotationsToRadians(2.0), Units.rotationsToRadians(2.0))
          .forever();

  private final AutoSegment defaultIntakeSecondCycle =
      Trailblazer.segment(
              AutoPoint.ofRed(
                      new Pose2d(
                          13.2,
                          FieldUtil.RED_DEPOT_TRENCH_CENTER.getY(),
                          Rotation2d.fromDegrees(-160)))
                  .withLinearConstraints(2.0, 2.0)
                  .withAngularConstraints(4.0, 3.0)
                  .withTransitionTolerance(new PoseErrorTolerance(2.0, 50)),
              AutoPoint.ofRed(
                      new Pose2d(
                          13.2, FieldUtil.RED_DEPOT_TRENCH_CENTER.getY(), Rotation2d.k180deg))
                  .withLinearConstraints(2.5, 8.0)
                  .withAngularConstraints(4.0, 3.0)
                  .withTransitionTolerance(new PoseErrorTolerance(0.2, 100)),
              AutoPoint.ofRed(
                      new Pose2d(
                          12.5, FieldUtil.RED_DEPOT_TRENCH_CENTER.getY(), Rotation2d.k180deg))
                  .withMarker(Markers.START_INTAKE_2)
                  .withLinearConstraints(3.0, 8.0)
                  .withAngularConstraints(2.0, 1.0)
                  .withTransitionTolerance(new PoseErrorTolerance(0.2, 100)),
              AutoPoint.ofRed(
                      new Pose2d(
                          11.5, FieldUtil.RED_DEPOT_TRENCH_CENTER.getY(), Rotation2d.k180deg))
                  .withLinearConstraints(4.5, 8.0)
                  .withAngularConstraints(0.5, 0.5)
                  .withTransitionTolerance(new PoseErrorTolerance(0.2, 100)),
              AutoPoint.ofRed(new Pose2d(10.628, 0.87, Rotation2d.fromDegrees(140)))
                  .withTransitionTolerance(new PoseErrorTolerance(0.4, 100)),
              AutoPoint.ofRed(new Pose2d(9.140, 1.417, Rotation2d.fromDegrees(130)))
                  .withTransitionTolerance(new PoseErrorTolerance(0.4, 100)),
              AutoPoint.ofRed(new Pose2d(8.7, 2.477, Rotation2d.fromDegrees(85)))
                  .withTransitionTolerance(new PoseErrorTolerance(0.4, 100))
                  .withMarker(Markers.MAKE_CLUSTER_MAP_DECISION),
              AutoPoint.ofRed(new Pose2d(8.500, 3.76, Rotation2d.fromDegrees(30)))
                  .withMarker(Markers.CANCEL_CLUSTER_MAP_CHECK)
                  .withTransitionTolerance(new PoseErrorTolerance(0.4, 100)),
              AutoPoint.ofRed(new Pose2d(10.07, 3.77, Rotation2d.fromDegrees(-15)))
                  .withTransitionTolerance(new PoseErrorTolerance(0.5, 100)),
              AutoPoint.ofRed(
                      new Pose2d(
                          10.2,
                          FieldUtil.RED_DEPOT_BUMP_CENTER.getY() - BUMP_OFFSET,
                          Rotation2d.fromDegrees(-50)))
                  .withTransitionTolerance(new PoseErrorTolerance(0.1, 100)),
              AutoPoint.ofRed(
                      new Pose2d(
                          SHOOT_X,
                          FieldUtil.RED_DEPOT_BUMP_CENTER.getY() - BUMP_OFFSET,
                          Rotation2d.kZero))
                  .withMarker(Markers.READY_TO_CROSS_BUMP)
                  .withTransitionTolerance(new PoseErrorTolerance(0.3, 100)))
          .withLinearConstraints(4.5, 8)
          .withAngularConstraints(Units.rotationsToRadians(4.0), Units.rotationsToRadians(4.0))
          .untilFinished(new PoseErrorTolerance(0.2, 3));

  private final AutoSegment intakeSecondCycleFar =
      Trailblazer.segment(
              AutoPoint.ofRed(new Pose2d(8.283, 2.477, Rotation2d.fromDegrees(85)))
                  .withTransitionTolerance(new PoseErrorTolerance(0.4, 100))
                  .withMarker(Markers.CANCEL_CLUSTER_MAP_CHECK),
              AutoPoint.ofRed(new Pose2d(8.300, 3.76, Rotation2d.fromDegrees(30)))
                  .withTransitionTolerance(new PoseErrorTolerance(0.4, 100)),
              AutoPoint.ofRed(new Pose2d(9.7, 3.77, Rotation2d.fromDegrees(-15)))
                  .withTransitionTolerance(new PoseErrorTolerance(0.5, 100)),
              AutoPoint.ofRed(
                      new Pose2d(
                          10.2,
                          FieldUtil.RED_DEPOT_BUMP_CENTER.getY() - BUMP_OFFSET,
                          Rotation2d.fromDegrees(-50)))
                  .withTransitionTolerance(new PoseErrorTolerance(0.1, 100)),
              AutoPoint.ofRed(
                      new Pose2d(
                          SHOOT_X,
                          FieldUtil.RED_DEPOT_BUMP_CENTER.getY() - BUMP_OFFSET,
                          Rotation2d.kZero))
                  .withMarker(Markers.READY_TO_CROSS_BUMP)
                  .withTransitionTolerance(new PoseErrorTolerance(0.3, 100)))
          .withLinearConstraints(4.5, 8)
          .withAngularConstraints(Units.rotationsToRadians(4.0), Units.rotationsToRadians(4.0))
          .untilFinished(new PoseErrorTolerance(0.2, 3));

  private final AutoSegment crossBumpToShootTwo =
      Trailblazer.segment(
              AutoPoint.ofRed(
                      new Pose2d(
                          SHOOT_X,
                          FieldUtil.RED_DEPOT_BUMP_CENTER.getY() - BUMP_OFFSET,
                          Rotation2d.kZero))
                  .withTransitionTolerance(new PoseErrorTolerance(0.1, 100))
                  .withLinearConstraints(4.5, 8)
                  .withMarker(Markers.START_SHOOT_RQ))
          .withLinearConstraints(4.5, 8)
          .withAngularConstraints(Units.rotationsToRadians(2.0), Units.rotationsToRadians(2.0))
          .forever();
  private final AutoSegment intakeThirdCycle =
      Trailblazer.segment(
              AutoPoint.ofRed(
                      new Pose2d(
                          13.2,
                          FieldUtil.RED_DEPOT_TRENCH_CENTER.getY(),
                          Rotation2d.fromDegrees(-160)))
                  .withLinearConstraints(2.0, 2.0)
                  .withAngularConstraints(4.0, 3.0)
                  .withTransitionTolerance(new PoseErrorTolerance(2.0, 50)),
              AutoPoint.ofRed(
                      new Pose2d(
                          13.2, FieldUtil.RED_DEPOT_TRENCH_CENTER.getY(), Rotation2d.k180deg))
                  .withLinearConstraints(2.5, 8.0)
                  .withAngularConstraints(4.0, 3.0)
                  .withTransitionTolerance(new PoseErrorTolerance(0.2, 100)),
              AutoPoint.ofRed(
                      new Pose2d(
                          12.5, FieldUtil.RED_DEPOT_TRENCH_CENTER.getY(), Rotation2d.k180deg))
                  .withLinearConstraints(3.0, 8.0)
                  .withAngularConstraints(2.0, 1.0)
                  .withMarker(Markers.START_INTAKE_3)
                  .withTransitionTolerance(new PoseErrorTolerance(0.2, 100)),
              AutoPoint.ofRed(
                      new Pose2d(
                          11.5, FieldUtil.RED_DEPOT_TRENCH_CENTER.getY(), Rotation2d.k180deg))
                  .withLinearConstraints(4.5, 8.0)
                  .withAngularConstraints(0.5, 0.5)
                  .withTransitionTolerance(new PoseErrorTolerance(0.2, 100)),
              AutoPoint.ofRed(new Pose2d(9.6, 1.717, Rotation2d.fromDegrees(130)))
                  .withTransitionTolerance(new PoseErrorTolerance(0.4, 100)),
              AutoPoint.ofRed(new Pose2d(8.7, 2.477, Rotation2d.kCCW_90deg))
                  .withTransitionTolerance(new PoseErrorTolerance(0.3, 100)),
              AutoPoint.ofRed(new Pose2d(8.600, 3.76, Rotation2d.kCCW_90deg))
                  .withTransitionTolerance(new PoseErrorTolerance(0.3, 100)),
              AutoPoint.ofRed(new Pose2d(9.31, 3.77, Rotation2d.fromDegrees(-32)))
                  .withTransitionTolerance(new PoseErrorTolerance(0.3, 100)),
              AutoPoint.ofRed(
                      new Pose2d(
                          9.7,
                          FieldUtil.RED_DEPOT_BUMP_CENTER.getY() - BUMP_OFFSET,
                          Rotation2d.kZero))
                  .withTransitionTolerance(new PoseErrorTolerance(0.3, 100)),
              AutoPoint.ofRed(
                      new Pose2d(
                          10.6,
                          FieldUtil.RED_DEPOT_BUMP_CENTER.getY() - BUMP_OFFSET,
                          Rotation2d.kZero))
                  .withTransitionTolerance(new PoseErrorTolerance(0.3, 100)),
              AutoPoint.ofRed(
                      new Pose2d(
                          SHOOT_X,
                          FieldUtil.RED_DEPOT_BUMP_CENTER.getY() - BUMP_OFFSET,
                          Rotation2d.kZero))
                  .withMarker(Markers.READY_TO_CROSS_BUMP)
                  .withTransitionTolerance(new PoseErrorTolerance(0.3, 100)))
          .withLinearConstraints(4.5, 8)
          .withAngularConstraints(Units.rotationsToRadians(4.0), Units.rotationsToRadians(3.0))
          .untilFinished(new PoseErrorTolerance(0.5, 3));

  private final AutoSegment crossBumpToShootThree =
      Trailblazer.segment(
              AutoPoint.ofRed(
                      new Pose2d(
                          SHOOT_X,
                          FieldUtil.RED_DEPOT_BUMP_CENTER.getY() - BUMP_OFFSET,
                          Rotation2d.kZero))
                  .withTransitionTolerance(new PoseErrorTolerance(0.1, 100))
                  .withLinearConstraints(4.5, 8)
                  .withMarker(Markers.START_SHOOT_RQ))
          .withLinearConstraints(4.5, 8)
          .withAngularConstraints(Units.rotationsToRadians(2.0), Units.rotationsToRadians(2.0))
          .forever();
  private AutoSegment stuckOnBall =
      StuckOnBallRecovery.getRecoverySegment(
          () -> robotManager.localization.getPose(),
          () -> Rotation2d.fromDegrees(robotManager.localization.imu.getPitch()),
          () -> Rotation2d.fromDegrees(robotManager.localization.imu.getRoll()));

  private NormalAutoState storedStuckOnBallState = NormalAutoState.INTAKE_FIRST_CYCLE;
  private AutoSegment storedStuckOnBallAutoSegment = intakeFirstCycle;
  private int storedStuckOnBallIndex = 0;

  // FOR SIM ONLY!!!
  private boolean firstStuckOnBall = false;
  private boolean secondStuckOnBall = false;

  public LeftNormalAuto(RobotManager robotManager, Trailblazer trailblazer) {
    super(NormalAutoState.INTAKE_FIRST_CYCLE, robotManager, trailblazer);

    this.bumpCrossingTracker = robotManager.localization.imu.bumpCrossingTracker;
  }

  @Override
  public Point getStartingPoint() {
    return Point.ofRed(new Pose2d(12.1, 0.47, Rotation2d.kCCW_90deg));
  }

  @Override
  protected void collectInputs() {
    super.collectInputs();
    if (!collisionEverDetected
        && getState() == NormalAutoState.INTAKE_FIRST_CYCLE
        && DriverStation.isEnabled()
        && robotManager.localization.imu.collisionDetected()) {
      collisionEverDetected = true;
      DogLog.log("LeftNormalAuto/CollisionDetected", collisionEverDetected);
    }
  }

  @Override
  protected NormalAutoState getNextState(NormalAutoState currentState) {
    if (FeatureFlags.UNBEACH_AUTO_IRL.getAsBoolean()
        || FeatureFlags.UNBEACH_AUTO_SIM_ONLY.getAsBoolean()) {
      switch (currentState) {
        case INTAKE_FIRST_CYCLE, INTAKE_THIRD_CYCLE -> {
          if (StuckOnBallRecovery.stuckOnBall(
              robotManager.localization.imu.getPitch(), robotManager.localization.imu.getRoll())) {
            return NormalAutoState.STUCK_ON_BALL_RECOVERY;
          }
        }
        default -> {}
      }
    }

    return switch (currentState) {
      case STUCK_ON_BALL_RECOVERY -> {
        if (!StuckOnBallRecovery.stuckOnBall(
            robotManager.localization.imu.getPitch(), robotManager.localization.imu.getRoll())) {
          yield storedStuckOnBallState;
        } else {
          yield currentState;
        }
      }
      case INTAKE_FIRST_CYCLE -> {
        if (trailblazer.passedMarker(Markers.READY_TO_CROSS_BUMP)) {
          robotManager.powerManager.idleRequest();
          yield NormalAutoState.CROSS_BUMP_TO_SHOOT_1;
        } else {
          yield currentState;
        }
      }
      case CROSS_BUMP_TO_SHOOT_1 -> {
        if (trailblazer.passedMarker(Markers.START_SHOOT_RQ)
            && bumpCrossingTracker.getState() == BumpCrossingState.FLAT_NOT_CROSSING) {
          yield NormalAutoState.SHOOT_1;
        } else {
          yield currentState;
        }
      }
      case SHOOT_1 -> {
        if ((timeout(2.0) && !robotManager.hopperManager.isShooting()) || timeout(3.25)) {
          yield NormalAutoState.DEFAULT_INTAKE_SECOND_CYCLE;
        } else {
          yield currentState;
        }
      }
      case DEFAULT_INTAKE_SECOND_CYCLE -> {
        if (trailblazer.passedMarker(Markers.READY_TO_CROSS_BUMP)) {
          yield NormalAutoState.CROSS_BUMP_TO_SHOOT_2;
        } else if (trailblazer.passedMarker(Markers.MAKE_CLUSTER_MAP_DECISION)
            && !trailblazer.passedMarker(Markers.CANCEL_CLUSTER_MAP_CHECK)) {

          Lane bestLane = robotManager.clusterMap.getBestClusterLane();
          yield switch (bestLane) {
            case LANE_1 -> NormalAutoState.INTAKE_SECOND_CYCLE_FAR;
            default -> currentState;
          };
        }
        yield currentState;
      }
      case INTAKE_SECOND_CYCLE_FAR -> {
        if (trailblazer.passedMarker(Markers.READY_TO_CROSS_BUMP)) {
          yield NormalAutoState.CROSS_BUMP_TO_SHOOT_2;
        } else {
          yield currentState;
        }
      }

      case CROSS_BUMP_TO_SHOOT_2 -> {
        if (trailblazer.passedMarker(Markers.START_SHOOT_RQ)
            && bumpCrossingTracker.getState() == BumpCrossingState.FLAT_NOT_CROSSING) {
          yield NormalAutoState.SHOOT_2;
        } else {
          yield currentState;
        }
      }
      case SHOOT_2 -> {
        if ((timeout(2.0) && !robotManager.hopperManager.isShooting()) || timeout(3.25)) {
          yield NormalAutoState.INTAKE_THIRD_CYCLE;
        } else {
          yield currentState;
        }
      }
      case INTAKE_THIRD_CYCLE -> {
        if (trailblazer.passedMarker(Markers.READY_TO_CROSS_BUMP)) {
          yield NormalAutoState.CROSS_BUMP_TO_SHOOT_3;
        } else {
          yield currentState;
        }
      }
      case CROSS_BUMP_TO_SHOOT_3 -> {
        if (trailblazer.passedMarker(Markers.START_SHOOT_RQ)
            && bumpCrossingTracker.getState() == BumpCrossingState.FLAT_NOT_CROSSING) {
          yield NormalAutoState.SHOOT_3;
        } else {
          yield currentState;
        }
      }
      case DONE -> {
        if (trailblazer.atGoal(robotManager.localization.getPose())) {
          yield NormalAutoState.DONE;
        } else {
          yield currentState;
        }
      }
      default -> currentState;
    };
  }

  @Override
  protected void whileInState(NormalAutoState newState) {
    if (getState() != NormalAutoState.STUCK_ON_BALL_RECOVERY) {
      storedStuckOnBallIndex = trailblazer.getCurrentPointIndex();
    }

    switch (newState) {
      case STUCK_ON_BALL_RECOVERY -> {
        trailblazer.setActiveSegment(stuckOnBall);

        if (FeatureFlags.UNBEACH_AUTO_SIM_ONLY.getAsBoolean()
            && timeout(1.0)
            && RobotBase.isSimulation()) {
          robotManager.localization.imu.setPitch(0.0);
          robotManager.localization.imu.setRoll(0.0);
        }
      }
      case INTAKE_FIRST_CYCLE -> {
        trailblazer.setActiveSegment(intakeFirstCycle);
        robotManager.intakeAutoRequest();
        robotManager.powerManager.firstAutoSegmentRequest();

        if (FeatureFlags.UNBEACH_AUTO_SIM_ONLY.getAsBoolean()
            && timeout(1.5)
            && RobotBase.isSimulation()
            && !firstStuckOnBall) {
          firstStuckOnBall = true;
          robotManager.localization.imu.setPitch(-30.0);
        }
        if (trailblazer.passedMarker(Markers.PRIORITIZE_INTAKE)) {
          robotManager.powerManager.prioritizeIntakeRequest();
        }
      }
      case CROSS_BUMP_TO_SHOOT_1 -> {
        trailblazer.setActiveSegment(crossBumpToShootOne);
        robotManager.cancelIntakeRequest();
        robotManager.shooter.prepareScoreRequest(SHOOTER_WARMUP_DISTANCE_FROM_HUB);
        if (RobotBase.isSimulation()) {
          if (timeout(0.2)) {
            robotManager.localization.imu.setPitch(-7.5);
            robotManager.localization.imu.setRoll(-7.5);
          }
          if (timeout(0.4)) {
            robotManager.localization.imu.setPitch(0.0);
            robotManager.localization.imu.setRoll(0.0);
          }
          if (timeout(0.6)) {
            robotManager.localization.imu.setPitch(7.5);
            robotManager.localization.imu.setRoll(7.5);
          }
          if (timeout(.7)) {
            robotManager.localization.imu.setPitch(0.0);
            robotManager.localization.imu.setRoll(0.0);
          }
        }
      }
      case SHOOT_1 -> robotManager.prepareScoreRequest();
      case DEFAULT_INTAKE_SECOND_CYCLE -> {
        trailblazer.setActiveSegment(defaultIntakeSecondCycle);
        if (trailblazer.passedMarker(Markers.START_INTAKE_2)) {

          robotManager.intakeAutoRequest();
        }
      }

      case INTAKE_SECOND_CYCLE_FAR -> {
        trailblazer.setActiveSegment(intakeSecondCycleFar);
        robotManager.intakeAutoRequest();

        if (FeatureFlags.UNBEACH_AUTO_SIM_ONLY.getAsBoolean()
            && timeout(1.5)
            && RobotBase.isSimulation()
            && !secondStuckOnBall) {
          secondStuckOnBall = true;
          robotManager.localization.imu.setPitch(-30.0);
        }
      }
      case CROSS_BUMP_TO_SHOOT_2 -> {
        trailblazer.setActiveSegment(crossBumpToShootTwo);
        robotManager.shooter.prepareScoreRequest(SHOOTER_WARMUP_DISTANCE_FROM_HUB);

        if (RobotBase.isSimulation()) {
          if (timeout(0.2)) {
            robotManager.localization.imu.setPitch(-7.5);
            robotManager.localization.imu.setRoll(-7.5);
          }
          if (timeout(0.4)) {
            robotManager.localization.imu.setPitch(0.0);
            robotManager.localization.imu.setRoll(0.0);
          }
          if (timeout(0.6)) {
            robotManager.localization.imu.setPitch(7.5);
            robotManager.localization.imu.setRoll(7.5);
          }
          if (timeout(.7)) {
            robotManager.localization.imu.setPitch(0.0);
            robotManager.localization.imu.setRoll(0.0);
          }
        }
      }

      case SHOOT_2 -> robotManager.prepareScoreRequest();
      case INTAKE_THIRD_CYCLE -> {
        trailblazer.setActiveSegment(intakeThirdCycle);
        if (trailblazer.passedMarker(Markers.START_INTAKE_3)) {

          robotManager.intakeAutoRequest();
        }
      }
      case CROSS_BUMP_TO_SHOOT_3 -> {
        trailblazer.setActiveSegment(crossBumpToShootTwo);
        robotManager.shooter.prepareScoreRequest(SHOOTER_WARMUP_DISTANCE_FROM_HUB);

        if (RobotBase.isSimulation()) {
          if (timeout(0.2)) {
            robotManager.localization.imu.setPitch(-7.5);
            robotManager.localization.imu.setRoll(-7.5);
          }
          if (timeout(0.4)) {
            robotManager.localization.imu.setPitch(0.0);
            robotManager.localization.imu.setRoll(0.0);
          }
          if (timeout(0.6)) {
            robotManager.localization.imu.setPitch(7.5);
            robotManager.localization.imu.setRoll(7.5);
          }
          if (timeout(.7)) {
            robotManager.localization.imu.setPitch(0.0);
            robotManager.localization.imu.setRoll(0.0);
          }
        }
      }
      case SHOOT_3 -> robotManager.prepareScoreRequest();
      case DONE -> {}
    }
  }

  @Override
  protected void beforeTransition(NormalAutoState oldState, NormalAutoState newState) {
    if (newState == NormalAutoState.STUCK_ON_BALL_RECOVERY) {
      storedStuckOnBallState = oldState;
      DogLog.log("Trailblazer/StoredStuckOnBall/State", storedStuckOnBallState);
      DogLog.log("Trailblazer/StoredStuckOnBall/Index", storedStuckOnBallIndex);
    }

    if (oldState == NormalAutoState.STUCK_ON_BALL_RECOVERY) {
      trailblazer.setActiveSegment(storedStuckOnBallAutoSegment, storedStuckOnBallIndex);
    }
  }

  @Override
  protected void afterTransition(NormalAutoState newState) {
    switch (newState) {
      case CROSS_BUMP_TO_SHOOT_1, CROSS_BUMP_TO_SHOOT_2, CROSS_BUMP_TO_SHOOT_3 -> {
        bumpCrossingTracker.bumpCrossRequest(
            Point.ofRed(
                new Pose2d(
                    13.25, FieldUtil.RED_DEPOT_BUMP_CENTER.getY() - BUMP_OFFSET, Rotation2d.kZero)),
            FmsUtil.isRedAlliance() ? Rotation2d.k180deg : Rotation2d.kZero);
      }
      default -> {}
    }

    switch (newState) {
      case INTAKE_FIRST_CYCLE -> {
        storedStuckOnBallAutoSegment = intakeFirstCycle;
        robotManager.homeDeployInAutoRequest();
        robotManager.homeShooterHoodRequest();
      }
      case CROSS_BUMP_TO_SHOOT_1, SHOOT_1 -> {
        storedStuckOnBallAutoSegment = crossBumpToShootOne;
      }
      case DEFAULT_INTAKE_SECOND_CYCLE -> {
        storedStuckOnBallAutoSegment = defaultIntakeSecondCycle;
        robotManager.idleRequest();
        robotManager.hopperManager.setWantsSafeStow(true);
      }
      case INTAKE_SECOND_CYCLE_FAR -> {
        storedStuckOnBallAutoSegment = intakeSecondCycleFar;
        robotManager.idleRequest();
      }
      case INTAKE_THIRD_CYCLE -> {
        storedStuckOnBallAutoSegment = intakeThirdCycle;
        robotManager.idleRequest();
        robotManager.hopperManager.setWantsSafeStow(true);
      }
      case CROSS_BUMP_TO_SHOOT_2 -> {
        storedStuckOnBallAutoSegment = crossBumpToShootTwo;
      }
      case SHOOT_2 -> {
        storedStuckOnBallAutoSegment = crossBumpToShootTwo;
        robotManager.cancelIntakeRequest();
      }
      case CROSS_BUMP_TO_SHOOT_3 -> {
        storedStuckOnBallAutoSegment = crossBumpToShootThree;
        robotManager.idleRequest();
      }
      case SHOOT_3 -> {
        storedStuckOnBallAutoSegment = crossBumpToShootThree;
        robotManager.cancelIntakeRequest();
      }
      case DONE -> {
        robotManager.idleRequest();
      }
      case STUCK_ON_BALL_RECOVERY -> {}
    }
  }

  private Point getCollisionPoint(Point point) {
    if (collisionEverDetected) {
      return Point.ofRed(
          new Pose2d(
              point.redPose().getX() + COLLISION_X_OFFSET,
              point.redPose().getY(),
              point.redPose().getRotation()));
    } else {
      return point;
    }
  }

  // Only use for lane 0 and 1 since we don't
  private Point getClusterShiftedPoint(Point point) {
    var targetCluster = robotManager.clusterMap.getBestClusterPose();

    if (targetCluster.isEmpty()) {
      return point;
    }

    Pose2d clusterPose = targetCluster.orElseThrow();
    Pose2d basePose = point.getPose();

    double clampedX =
        MathUtil.clamp(
            clusterPose.getX(),
            basePose.getX() - MAX_CLUSTER_MAP_OFFSET,
            basePose.getX() + MAX_CLUSTER_MAP_OFFSET);

    return FmsUtil.isRedAlliance()
        ? Point.ofRed(new Pose2d(clampedX, basePose.getY(), basePose.getRotation()))
        : Point.ofBlue(new Pose2d(clampedX, basePose.getY(), basePose.getRotation()));
  }
}
