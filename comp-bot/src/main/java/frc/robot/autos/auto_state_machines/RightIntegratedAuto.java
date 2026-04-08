package frc.robot.autos.auto_state_machines;

import com.team581.autos.Point;
import com.team581.autos.StuckOnBallRecovery;
import com.team581.math.PoseErrorTolerance;
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
import frc.robot.autos.auto_state_machines.auto_state.IntegratedAutoState;
import frc.robot.cluster_map.Lane;
import frc.robot.config.FeatureFlags;
import frc.robot.robot_manager.RobotManager;

public class RightIntegratedAuto extends BaseImperativeAuto<IntegratedAutoState> {

  public enum Markers {
    START_SHOOT_RQ,
    CANCEL_INTAKE_RQ,
    READY_TO_SHOOT_FOR_2,
    CHECK_CLUSTER_MAP_TRENCH,
    MAKE_CLUSTER_MAP_DECISION,
    CANCEL_CLUSTER_MAP_CHECK
  }

  private BumpCrossingTracker bumpCrossingTracker;

  private static final double BUMP_OFFSET = -0.15;
  private static final double COLLISION_X_OFFSET = 0.5;
  private static final double MAX_CLUSTER_MAP_OFFSET = 0.35;

  private boolean collisionEverDetected = false;

  private final AutoSegment intakeAcrossMidline =
      Trailblazer.segment(
              AutoPoint.ofRed(
                      new Pose2d(
                          10.489, FieldUtil.RED_OUTPOST_TRENCH_CENTER.getY(), Rotation2d.kCW_90deg))
                  .withTransitionTolerance(new PoseErrorTolerance(0.4, 100)),
              AutoPoint.of(
                      () ->
                          getCollisionPoint(
                              Point.ofRed(new Pose2d(8.640, 6.653, Rotation2d.kCW_90deg))))
                  .withTransitionTolerance(new PoseErrorTolerance(0.3, 100)),
              AutoPoint.of(
                      () ->
                          getCollisionPoint(
                              Point.ofRed(new Pose2d(7.983, 5.593, Rotation2d.kCW_90deg))))
                  .withTransitionTolerance(new PoseErrorTolerance(0.3, 100)),
              AutoPoint.of(
                      () ->
                          getCollisionPoint(
                              Point.ofRed(new Pose2d(8.280, 3.875, Rotation2d.fromDegrees(-50.0)))))
                  .withTransitionTolerance(new PoseErrorTolerance(0.3, 100)),
              AutoPoint.ofRed(new Pose2d(9.31, 4.31, Rotation2d.kCCW_90deg))
                  .withTransitionTolerance(new PoseErrorTolerance(0.3, 100)),
              AutoPoint.ofRed(
                      new Pose2d(
                          9.975,
                          FieldUtil.RED_OUTPOST_BUMP_CENTER.getY() + BUMP_OFFSET,
                          Rotation2d.kCCW_90deg))
                  .withTransitionTolerance(new PoseErrorTolerance(0.3, 100)),
              AutoPoint.ofRed(
                      new Pose2d(
                          11.174,
                          FieldUtil.RED_OUTPOST_BUMP_CENTER.getY() + BUMP_OFFSET,
                          Rotation2d.kCCW_90deg))
                  .withTransitionTolerance(new PoseErrorTolerance(0.2, 30))
                  .withMarker(Markers.CANCEL_INTAKE_RQ))
          .withLinearConstraints(4.5, 8)
          .withAngularConstraints(Units.rotationsToRadians(4.0), Units.rotationsToRadians(4.0))
          .untilFinished(new PoseErrorTolerance(0.2, 3));

  private final AutoSegment driveBackAndShootOne =
      Trailblazer.segment(
              AutoPoint.of(
                      () ->
                          bumpCrossingTracker.getPoint(
                              Point.ofRed(
                                  new Pose2d(
                                      13.709,
                                      FieldUtil.RED_OUTPOST_BUMP_CENTER.getY() + BUMP_OFFSET,
                                      Rotation2d.kCCW_90deg)),
                              Point.ofRed(new Pose2d(13.9, 5.443, Rotation2d.kCCW_90deg))))
                  .withTransitionTolerance(new PoseErrorTolerance(0.3, 100))
                  .withLinearConstraints(4.5, 8),
              AutoPoint.ofRed(
                      new Pose2d(
                          14.0,
                          FieldUtil.RED_OUTPOST_BUMP_CENTER.getY() + BUMP_OFFSET,
                          Rotation2d.fromDegrees(40.0)))
                  .withTransitionTolerance(new PoseErrorTolerance(1.75, 100))
                  .withLinearConstraints(4.5, 8)
                  .withMarker(Markers.START_SHOOT_RQ))
          .withLinearConstraints(4.5, 8)
          .withAngularConstraints(Units.rotationsToRadians(2.0), Units.rotationsToRadians(2.0))
          .untilFinished(new PoseErrorTolerance(0.3, 100));

  private final AutoSegment defaultSecondSegment =
      Trailblazer.segment(
              AutoPoint.ofRed(
                      new Pose2d(
                          13.709, FieldUtil.RED_OUTPOST_TRENCH_CENTER.getY(), Rotation2d.k180deg))
                  .withLinearConstraints(3.0, 8.0)
                  .withTransitionTolerance(new PoseErrorTolerance(0.3, 100)),
              AutoPoint.ofRed(
                      new Pose2d(
                          12.5, FieldUtil.RED_OUTPOST_TRENCH_CENTER.getY(), Rotation2d.k180deg))
                  .withLinearConstraints(3.0, 8.0)
                  .withTransitionTolerance(new PoseErrorTolerance(0.3, 100)),
              AutoPoint.ofRed(
                      new Pose2d(
                          11.5,
                          FieldUtil.RED_OUTPOST_TRENCH_CENTER.getY(),
                          Rotation2d.fromDegrees(180 + 20)))
                  .withLinearConstraints(4.5, 8.0)
                  .withTransitionTolerance(new PoseErrorTolerance(0.3, 100)),
              AutoPoint.ofRed(
                      new Pose2d(
                          10.5,
                          FieldUtil.RED_OUTPOST_TRENCH_CENTER.getY(),
                          Rotation2d.fromDegrees(180 + 90)))
                  .withMarker(Markers.CHECK_CLUSTER_MAP_TRENCH)
                  .withLinearConstraints(2.0, 2.0)
                  .withTransitionTolerance(new PoseErrorTolerance(0.2, 100)),
              AutoPoint.ofRed(new Pose2d(9.5, 6.814, Rotation2d.kCW_90deg))
                  .withMarker(Markers.MAKE_CLUSTER_MAP_DECISION)
                  .withTransitionTolerance(new PoseErrorTolerance(0.3, 100)),
              AutoPoint.of(
                      () ->
                          getClusterShiftedPoint(
                              Point.ofRed(new Pose2d(9.5, 4.4, Rotation2d.kCW_90deg))))
                  .withTransitionTolerance(new PoseErrorTolerance(0.6, 100))
                  .withAngularConstraints(
                      Units.rotationsToRadians(2.5), Units.rotationsToRadians(2.0)),
              AutoPoint.of(
                      () ->
                          getClusterShiftedPoint(
                              Point.ofRed(new Pose2d(10.575, 4.4, Rotation2d.kCCW_90deg))))
                  .withTransitionTolerance(new PoseErrorTolerance(0.7, 100))
                  .withArcMidpoint(Point.ofRed(new Pose2d(9.985, 3.89, Rotation2d.kZero)))
                  .withAngularConstraints(
                      Units.rotationsToRadians(2.5), Units.rotationsToRadians(2.0)),
              AutoPoint.ofRed(
                      new Pose2d(
                          10.575,
                          FieldUtil.RED_OUTPOST_BUMP_CENTER.getY() + BUMP_OFFSET,
                          Rotation2d.kCCW_90deg))
                  .withTransitionTolerance(new PoseErrorTolerance(0.3, 100))
                  .withMarker(Markers.CANCEL_INTAKE_RQ))
          .withLinearConstraints(4.5, 8.0)
          .withAngularConstraints(Units.rotationsToRadians(3.5), Units.rotationsToRadians(3.0))
          .untilFinished(new PoseErrorTolerance(0.3, 100));

  private final AutoSegment trenchSegment =
      Trailblazer.segment(
              AutoPoint.ofRed(
                      new Pose2d(
                          10.5,
                          FieldUtil.RED_OUTPOST_TRENCH_CENTER.getY() + Units.inchesToMeters(3),
                          Rotation2d.fromDegrees(170)))
                  .withLinearConstraints(3.0, 2.0)
                  .withTransitionTolerance(new PoseErrorTolerance(0.2, 100)),
              AutoPoint.ofRed(
                      new Pose2d(
                          8.5,
                          FieldUtil.RED_OUTPOST_TRENCH_CENTER.getY() + Units.inchesToMeters(3),
                          Rotation2d.fromDegrees(170)))
                  .withLinearConstraints(3.0, 2.0)
                  .withTransitionTolerance(new PoseErrorTolerance(0.2, 100)),
              AutoPoint.ofRed(new Pose2d(8.1, 6.814, Rotation2d.kCW_90deg))
                  .withTransitionTolerance(new PoseErrorTolerance(0.3, 100)),
              AutoPoint.ofRed(new Pose2d(9.0, 4.4, Rotation2d.kCW_90deg))
                  .withTransitionTolerance(new PoseErrorTolerance(0.6, 100))
                  .withAngularConstraints(
                      Units.rotationsToRadians(2.0), Units.rotationsToRadians(2.0)),
              AutoPoint.ofRed(new Pose2d(10.575, 4.4, Rotation2d.kCCW_90deg))
                  .withTransitionTolerance(new PoseErrorTolerance(0.7, 100))
                  .withArcMidpoint(Point.ofRed(new Pose2d(9.7875, 3.89, Rotation2d.kZero)))
                  .withAngularConstraints(
                      Units.rotationsToRadians(2.0), Units.rotationsToRadians(2.0)),
              AutoPoint.ofRed(
                      new Pose2d(
                          10.575,
                          FieldUtil.RED_OUTPOST_BUMP_CENTER.getY() + BUMP_OFFSET,
                          Rotation2d.kCCW_90deg))
                  .withTransitionTolerance(new PoseErrorTolerance(0.3, 100))
                  .withMarker(Markers.CANCEL_INTAKE_RQ))
          .withLinearConstraints(4.5, 8.0)
          .withAngularConstraints(Units.rotationsToRadians(3.0), Units.rotationsToRadians(3.0))
          .untilFinished(new PoseErrorTolerance(0.3, 100));

  private final AutoSegment lane1Segment =
      Trailblazer.segment(
              AutoPoint.of(
                      () ->
                          getClusterShiftedPoint(
                              Point.ofRed(new Pose2d(8.75, 6.814, Rotation2d.kCW_90deg))))
                  .withTransitionTolerance(new PoseErrorTolerance(0.3, 100)),
              AutoPoint.of(
                      () ->
                          getClusterShiftedPoint(
                              Point.ofRed(new Pose2d(8.75, 4.4, Rotation2d.kCW_90deg))))
                  .withTransitionTolerance(new PoseErrorTolerance(0.6, 100))
                  .withAngularConstraints(
                      Units.rotationsToRadians(2.0), Units.rotationsToRadians(2.0)),
              AutoPoint.ofRed(new Pose2d(10.575, 4.4, Rotation2d.kCCW_90deg))
                  .withTransitionTolerance(new PoseErrorTolerance(0.7, 100))
                  .withArcMidpoint(Point.ofRed(new Pose2d(9.6625, 3.89, Rotation2d.kZero)))
                  .withAngularConstraints(
                      Units.rotationsToRadians(2.0), Units.rotationsToRadians(2.0)),
              AutoPoint.ofRed(
                      new Pose2d(
                          10.575,
                          FieldUtil.RED_OUTPOST_BUMP_CENTER.getY() + BUMP_OFFSET,
                          Rotation2d.kCCW_90deg))
                  .withTransitionTolerance(new PoseErrorTolerance(0.3, 100))
                  .withMarker(Markers.CANCEL_INTAKE_RQ))
          .withLinearConstraints(4.5, 4.0)
          .withAngularConstraints(Units.rotationsToRadians(3.0), Units.rotationsToRadians(3.0))
          .untilFinished(new PoseErrorTolerance(0.3, 100));

  private final AutoSegment lane2Segment =
      Trailblazer.segment(
              AutoPoint.ofRed(new Pose2d(7.983, 6.814, Rotation2d.kCW_90deg))
                  .withTransitionTolerance(new PoseErrorTolerance(0.3, 100)),
              AutoPoint.ofRed(new Pose2d(7.983, 4.4, Rotation2d.kCW_90deg))
                  .withTransitionTolerance(new PoseErrorTolerance(0.6, 100))
                  .withAngularConstraints(
                      Units.rotationsToRadians(2.0), Units.rotationsToRadians(2.0)),
              AutoPoint.ofRed(new Pose2d(10.575, 4.4, Rotation2d.kCCW_90deg))
                  .withTransitionTolerance(new PoseErrorTolerance(0.7, 100))
                  .withArcMidpoint(Point.ofRed(new Pose2d(9.35, 3.4, Rotation2d.kZero)))
                  .withAngularConstraints(
                      Units.rotationsToRadians(2.0), Units.rotationsToRadians(2.0)),
              AutoPoint.ofRed(
                      new Pose2d(
                          10.575,
                          FieldUtil.RED_OUTPOST_BUMP_CENTER.getY() + BUMP_OFFSET,
                          Rotation2d.kCCW_90deg))
                  .withTransitionTolerance(new PoseErrorTolerance(0.3, 100))
                  .withMarker(Markers.CANCEL_INTAKE_RQ))
          .withLinearConstraints(4.5, 4.0)
          .withAngularConstraints(Units.rotationsToRadians(3.0), Units.rotationsToRadians(3.0))
          .untilFinished(new PoseErrorTolerance(0.3, 100));

  private final AutoSegment driveBackAndShootTwo =
      Trailblazer.segment(
              AutoPoint.ofRed(
                      new Pose2d(
                          10.8,
                          FieldUtil.RED_OUTPOST_BUMP_CENTER.getY() + BUMP_OFFSET,
                          Rotation2d.kCCW_90deg))
                  .withAngularConstraints(
                      Units.rotationsToRadians(4.0), Units.rotationsToRadians(4.0))
                  .withTransitionTolerance(new PoseErrorTolerance(0.3, 10)),
              AutoPoint.of(
                      () ->
                          bumpCrossingTracker.getPoint(
                              Point.ofRed(
                                  new Pose2d(
                                      13.709,
                                      FieldUtil.RED_OUTPOST_BUMP_CENTER.getY() + BUMP_OFFSET,
                                      Rotation2d.kCCW_90deg)),
                              Point.ofRed(new Pose2d(13.9, 5.443, Rotation2d.kCCW_90deg))))
                  .withTransitionTolerance(new PoseErrorTolerance(0.3, 100))
                  .withLinearConstraints(4.5, 8),
              AutoPoint.ofRed(
                      new Pose2d(
                          14.0,
                          FieldUtil.RED_OUTPOST_BUMP_CENTER.getY() + BUMP_OFFSET,
                          Rotation2d.fromDegrees(40)))
                  .withTransitionTolerance(new PoseErrorTolerance(2.0, 100))
                  .withLinearConstraints(4.5, 8)
                  .withMarker(Markers.START_SHOOT_RQ))
          .withLinearConstraints(4.5, 8)
          .withAngularConstraints(Units.rotationsToRadians(2.0), Units.rotationsToRadians(3))
          .untilFinished(new PoseErrorTolerance(0.5, 100));

  private final AutoSegment driveBackToNeutralZone =
      Trailblazer.segment(
              AutoPoint.ofRed(
                      new Pose2d(
                          13.709, FieldUtil.RED_OUTPOST_TRENCH_CENTER.getY(), Rotation2d.k180deg))
                  .withTransitionTolerance(new PoseErrorTolerance(0.3, 100)),
              AutoPoint.ofRed(
                      new Pose2d(
                          13.0, FieldUtil.RED_OUTPOST_TRENCH_CENTER.getY(), Rotation2d.k180deg))
                  .withTransitionTolerance(new PoseErrorTolerance(0.3, 100)),
              AutoPoint.ofRed(new Pose2d(10.174, 7.28, Rotation2d.k180deg))
                  .withTransitionTolerance(new PoseErrorTolerance(0.3, 100)),
              AutoPoint.ofRed(new Pose2d(9.8, 5.708, Rotation2d.k180deg))
                  .withTransitionTolerance(new PoseErrorTolerance(0.3, 100)))
          .withLinearConstraints(4.5, 8)
          .withAngularConstraints(Units.rotationsToRadians(1.0), Units.rotationsToRadians(2))
          .untilFinished(new PoseErrorTolerance(0.3, 3));

  private AutoSegment stuckOnBall =
      StuckOnBallRecovery.getRecoverySegment(
          () -> robotManager.localization.getPose(),
          () -> Rotation2d.fromDegrees(robotManager.localization.imu.getPitch()),
          () -> Rotation2d.fromDegrees(robotManager.localization.imu.getRoll()));

  private IntegratedAutoState storedStuckOnBallState = IntegratedAutoState.INTAKE_ACROSS_MIDLINE;
  private AutoSegment storedStuckOnBallAutoSegment = intakeAcrossMidline;
  private int storedStuckOnBallIndex = 0;

  // FOR SIM ONLY!!!
  private boolean firstStuckOnBall = false;
  private boolean secondStuckOnBall = false;

  public RightIntegratedAuto(RobotManager robotManager, Trailblazer trailblazer) {
    super(IntegratedAutoState.INTAKE_ACROSS_MIDLINE, robotManager, trailblazer);

    this.bumpCrossingTracker = robotManager.localization.imu.bumpCrossingTracker;
  }

  @Override
  public Point getStartingPoint() {
    return Point.ofRed(new Pose2d(12.1, 7.6, Rotation2d.kCW_90deg));
  }

  @Override
  protected void collectInputs() {
    super.collectInputs();
    if (getState() == IntegratedAutoState.INTAKE_ACROSS_MIDLINE
        && DriverStation.isEnabled()
        && robotManager.localization.imu.collisionDetected()) {
      collisionEverDetected = true;
    }

    DogLog.log("RightIntegratedAuto/CollisionDetected", collisionEverDetected);
  }

  @Override
  protected IntegratedAutoState getNextState(IntegratedAutoState currentState) {
    if (FeatureFlags.UNBEACH_AUTO.getAsBoolean()) {
      switch (currentState) {
        case INTAKE_ACROSS_MIDLINE,
            DEFAULT_SECOND_INTAKE_SEGMENT,
            INTAKE_LANE_1,
            INTAKE_LANE_2,
            INTAKE_TRENCH_LANE -> {
          if (StuckOnBallRecovery.stuckOnBall(
              robotManager.localization.imu.getPitch(), robotManager.localization.imu.getRoll())) {
            return IntegratedAutoState.STUCK_ON_BALL_RECOVERY;
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
      case INTAKE_ACROSS_MIDLINE -> {
        if (trailblazer.passedMarker(Markers.CANCEL_INTAKE_RQ)) {
          yield IntegratedAutoState.DRIVE_BACK_1;
        } else {
          yield currentState;
        }
      }
      case DRIVE_BACK_1 -> {
        if (trailblazer.passedMarker(Markers.START_SHOOT_RQ)) {
          yield IntegratedAutoState.SHOOT_1;
        } else {
          yield currentState;
        }
      }
      case SHOOT_1 -> {
        if ((timeout(1.0) && !robotManager.hopperManager.isShooting()) || timeout(5.0)) {
          yield IntegratedAutoState.DEFAULT_SECOND_INTAKE_SEGMENT;
        } else {
          yield currentState;
        }
      }
      case DEFAULT_SECOND_INTAKE_SEGMENT -> {
        if (trailblazer.atGoal(robotManager.localization.getPose())
            && trailblazer.passedMarker(Markers.CANCEL_INTAKE_RQ)) {
          yield IntegratedAutoState.DRIVE_BACK_2;
        } else if (trailblazer.passedMarker(Markers.MAKE_CLUSTER_MAP_DECISION)
            && !trailblazer.passedMarker(Markers.CANCEL_CLUSTER_MAP_CHECK)) {

          Lane bestLane = robotManager.clusterMap.getBestClusterLane();
          yield switch (bestLane) {
            case LANE_0 -> currentState;
            case LANE_1 -> IntegratedAutoState.INTAKE_LANE_1;
            case LANE_2 -> IntegratedAutoState.INTAKE_LANE_2;
            case TRENCH -> IntegratedAutoState.INTAKE_TRENCH_LANE;
            default -> currentState;
          };
        } else if (trailblazer.passedMarker(Markers.CHECK_CLUSTER_MAP_TRENCH)) {
          if (robotManager.clusterMap.hasHighValueTrenchCluster()) {
            yield IntegratedAutoState.INTAKE_TRENCH_LANE;
          }
        }
        yield currentState;
      }
      case INTAKE_LANE_1, INTAKE_LANE_2, INTAKE_TRENCH_LANE -> {
        if (trailblazer.atGoal(robotManager.localization.getPose())
            && trailblazer.passedMarker(Markers.CANCEL_INTAKE_RQ)) {
          yield IntegratedAutoState.DRIVE_BACK_2;
        } else {
          yield currentState;
        }
      }
      case DRIVE_BACK_2 -> {
        if (trailblazer.passedMarker(Markers.START_SHOOT_RQ)) {
          yield IntegratedAutoState.SHOOT_2;
        } else {
          yield currentState;
        }
      }
      case SHOOT_2 -> {
        if ((timeout(1.0) && !robotManager.hopperManager.isShooting()) || timeout(5.0)) {
          yield IntegratedAutoState.DRIVE_BACK_TO_NEUTRAL_ZONE;
        } else {
          yield currentState;
        }
      }
      case DRIVE_BACK_TO_NEUTRAL_ZONE -> {
        if (trailblazer.atGoal(robotManager.localization.getPose())) {
          yield IntegratedAutoState.DONE;
        } else {
          yield currentState;
        }
      }
      case DONE -> {
        if (trailblazer.atGoal(robotManager.localization.getPose())) {
          yield IntegratedAutoState.DONE;
        } else {
          yield currentState;
        }
      }
      default -> currentState;
    };
  }

  @Override
  protected void whileInState(IntegratedAutoState newState) {
    if (getState() != IntegratedAutoState.STUCK_ON_BALL_RECOVERY) {
      storedStuckOnBallIndex = trailblazer.getCurrentPointIndex();
    }
    DogLog.log("Trailblazer/StoredStuckOnBall/State", storedStuckOnBallState);
    DogLog.log("Trailblazer/StoredStuckOnBall/Index", storedStuckOnBallIndex);

    switch (newState) {
      case STUCK_ON_BALL_RECOVERY -> {
        trailblazer.setActiveSegment(stuckOnBall);

        if (FeatureFlags.UNBEACH_AUTO.getAsBoolean() && timeout(1.0) && RobotBase.isSimulation()) {
          robotManager.localization.imu.setPitch(0.0);
          robotManager.localization.imu.setRoll(0.0);
        }
      }
      case INTAKE_ACROSS_MIDLINE -> {
        trailblazer.setActiveSegment(intakeAcrossMidline);
        robotManager.intakeAutoRequest();
        robotManager.powerManager.firstAutoSegmentRequest();

        if (FeatureFlags.UNBEACH_AUTO.getAsBoolean()
            && timeout(1.5)
            && RobotBase.isSimulation()
            && !firstStuckOnBall) {
          firstStuckOnBall = true;
          robotManager.localization.imu.setPitch(-30.0);
        }
      }
      case DRIVE_BACK_1 -> {
        trailblazer.setActiveSegment(driveBackAndShootOne);
        robotManager.cancelIntakeRequest();
      }
      case SHOOT_1 -> robotManager.prepareScoreRequest();
      case DEFAULT_SECOND_INTAKE_SEGMENT -> {
        trailblazer.setActiveSegment(defaultSecondSegment);
        robotManager.intakeAutoRequest();
      }

      case INTAKE_LANE_1 -> {
        trailblazer.setActiveSegment(lane1Segment);
        robotManager.intakeAutoRequest();

        if (FeatureFlags.UNBEACH_AUTO.getAsBoolean()
            && timeout(1.5)
            && RobotBase.isSimulation()
            && !secondStuckOnBall) {
          secondStuckOnBall = true;
          robotManager.localization.imu.setPitch(-30.0);
        }
      }
      case INTAKE_LANE_2 -> {
        trailblazer.setActiveSegment(lane2Segment);
        robotManager.intakeAutoRequest();

        if (FeatureFlags.UNBEACH_AUTO.getAsBoolean()
            && timeout(1.5)
            && RobotBase.isSimulation()
            && !secondStuckOnBall) {
          secondStuckOnBall = true;
          robotManager.localization.imu.setPitch(-30.0);
        }
      }
      case INTAKE_TRENCH_LANE -> {
        trailblazer.setActiveSegment(trenchSegment);
        robotManager.intakeAutoRequest();
      }
      case DRIVE_BACK_2 -> {
        trailblazer.setActiveSegment(driveBackAndShootTwo);
      }
      case SHOOT_2 -> robotManager.prepareScoreRequest();
      case DRIVE_BACK_TO_NEUTRAL_ZONE -> {
        trailblazer.setActiveSegment(driveBackToNeutralZone);
        robotManager.intakeAutoRequest();
      }
      case DONE -> {}
    }
  }

  @Override
  protected void beforeTransition(IntegratedAutoState oldState, IntegratedAutoState newState) {
    if (newState == IntegratedAutoState.STUCK_ON_BALL_RECOVERY) {
      storedStuckOnBallState = oldState;
    }

    if (oldState == IntegratedAutoState.STUCK_ON_BALL_RECOVERY) {
      trailblazer.setActiveSegment(storedStuckOnBallAutoSegment, storedStuckOnBallIndex);
    }
  }

  @Override
  protected void afterTransition(IntegratedAutoState newState) {
    switch (newState) {
      case INTAKE_ACROSS_MIDLINE -> {
        storedStuckOnBallAutoSegment = intakeAcrossMidline;
        robotManager.homeDeployInAutoRequest();
        robotManager.homeShooterHoodRequest();
      }
      case DRIVE_BACK_1, SHOOT_1 -> {
        storedStuckOnBallAutoSegment = driveBackAndShootOne;
      }
      case DEFAULT_SECOND_INTAKE_SEGMENT -> {
        storedStuckOnBallAutoSegment = defaultSecondSegment;
        robotManager.idleRequest();
      }
      case INTAKE_LANE_1 -> {
        storedStuckOnBallAutoSegment = lane1Segment;
        robotManager.idleRequest();
      }
      case INTAKE_LANE_2 -> {
        storedStuckOnBallAutoSegment = lane2Segment;
        robotManager.idleRequest();
      }
      case INTAKE_TRENCH_LANE -> {
        storedStuckOnBallAutoSegment = trenchSegment;
        robotManager.idleRequest();
      }
      case DRIVE_BACK_2 -> {
        storedStuckOnBallAutoSegment = driveBackAndShootTwo;
      }
      case SHOOT_2 -> {
        storedStuckOnBallAutoSegment = driveBackAndShootTwo;
        robotManager.cancelIntakeRequest();
      }
      case DRIVE_BACK_TO_NEUTRAL_ZONE -> {
        storedStuckOnBallAutoSegment = driveBackToNeutralZone;
        robotManager.idleRequest();
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
