package frc.robot.autos.auto_state_machines;

import com.team581.autos.Point;
import com.team581.math.PoseErrorTolerance;
import com.team581.mechanisms.imu.BumpCrossingTracker;
import com.team581.trailblazer.AutoPoint;
import com.team581.trailblazer.Trailblazer;
import com.team581.trailblazer.segments.AutoSegment;
import com.team581.util.FieldUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.autos.BaseImperativeAuto;
import frc.robot.autos.auto_state_machines.auto_state.IntegratedAutoState;
import frc.robot.robot_manager.RobotManager;

public class RightIntegratedAuto extends BaseImperativeAuto<IntegratedAutoState> {

  public enum Markers {
    START_SHOOT_RQ,
    CANCEL_INTAKE_RQ,
    READY_TO_SHOOT_FOR_2
  }

  private BumpCrossingTracker bumpCrossingTracker;

  private final AutoSegment intakeAcrossMidline =
      Trailblazer.segment(
              AutoPoint.ofRed(
                      new Pose2d(
                          10.489, FieldUtil.RED_OUTPOST_TRENCH_CENTER.getY(), Rotation2d.k180deg))
                  .withTransitionTolerance(new PoseErrorTolerance(0.4, 100)),
              AutoPoint.ofRed(new Pose2d(8.640, 6.653, Rotation2d.fromDegrees(-134.0)))
                  .withTransitionTolerance(new PoseErrorTolerance(0.3, 100)),
              AutoPoint.ofRed(new Pose2d(7.983, 5.593, Rotation2d.fromDegrees(-86.0)))
                  .withTransitionTolerance(new PoseErrorTolerance(0.3, 100)),
              AutoPoint.ofRed(new Pose2d(8.280, 4.475, Rotation2d.fromDegrees(-50.0)))
                  .withTransitionTolerance(new PoseErrorTolerance(0.3, 100)),
              AutoPoint.ofRed(new Pose2d(9.31, 4.31, Rotation2d.kCCW_90deg))
                  .withTransitionTolerance(new PoseErrorTolerance(0.3, 100)),
              AutoPoint.ofRed(
                      new Pose2d(
                          9.975, FieldUtil.RED_OUTPOST_BUMP_CENTER.getY(), Rotation2d.kCCW_90deg))
                  .withTransitionTolerance(new PoseErrorTolerance(0.3, 100)),
              AutoPoint.ofRed(
                      new Pose2d(
                          11.174, FieldUtil.RED_OUTPOST_BUMP_CENTER.getY(), Rotation2d.kCCW_90deg))
                  .withTransitionTolerance(new PoseErrorTolerance(0.2, 30))
                  .withMarker(Markers.CANCEL_INTAKE_RQ))
          .withLinearConstraints(2.5, 8)
          .withAngularConstraints(Units.rotationsToRadians(3.0), Units.rotationsToRadians(3.0))
          .untilFinished(new PoseErrorTolerance(0.2, 3));

  private final AutoSegment driveBackAndShootOne =
      Trailblazer.segment(
              AutoPoint.of(
                      () ->
                          bumpCrossingTracker.getPoint(
                              Point.ofRed(
                                  new Pose2d(
                                      13.709,
                                      FieldUtil.RED_OUTPOST_BUMP_CENTER.getY(),
                                      Rotation2d.kCCW_90deg)),
                              Point.ofRed(new Pose2d(13.9, 5.593, Rotation2d.kCCW_90deg))))
                  .withTransitionTolerance(new PoseErrorTolerance(0.3, 100))
                  .withLinearConstraints(4.5, 8),
              AutoPoint.ofRed(
                      new Pose2d(
                          14.0,
                          FieldUtil.RED_OUTPOST_BUMP_CENTER.getY(),
                          Rotation2d.fromDegrees(40.0)))
                  .withTransitionTolerance(new PoseErrorTolerance(0.5))
                  .withLinearConstraints(4.5, 8)
                  .withMarker(Markers.START_SHOOT_RQ))
          .withLinearConstraints(2.5, 8)
          .withAngularConstraints(Units.rotationsToRadians(2.0), Units.rotationsToRadians(2.0))
          .untilFinished(new PoseErrorTolerance(0.3, 100));

  private final AutoSegment intakeBehindHub =
      Trailblazer.segment(
              AutoPoint.ofRed(
                      new Pose2d(
                          13.709, FieldUtil.RED_OUTPOST_TRENCH_CENTER.getY(), Rotation2d.k180deg))
                  .withTransitionTolerance(new PoseErrorTolerance(0.3, 100)),
              AutoPoint.ofRed(
                      new Pose2d(
                          13.0, FieldUtil.RED_OUTPOST_TRENCH_CENTER.getY(), Rotation2d.k180deg))
                  .withTransitionTolerance(new PoseErrorTolerance(0.3, 100)),
              AutoPoint.ofRed(
                      new Pose2d(
                          11.0, FieldUtil.RED_OUTPOST_TRENCH_CENTER.getY(), Rotation2d.k180deg))
                  .withTransitionTolerance(new PoseErrorTolerance(0.3, 100)),
              AutoPoint.ofRed(new Pose2d(9.5, 6.814, Rotation2d.kCW_90deg))
                  .withTransitionTolerance(new PoseErrorTolerance(0.3, 100)),
              AutoPoint.ofRed(new Pose2d(9.5, 4.4, Rotation2d.kCW_90deg))
                  .withTransitionTolerance(new PoseErrorTolerance(0.6, 100))
                  .withAngularConstraints(
                      Units.rotationsToRadians(2.0), Units.rotationsToRadians(2.0)),
              AutoPoint.ofRed(new Pose2d(10.575, 4.4, Rotation2d.kCCW_90deg))
                  .withTransitionTolerance(new PoseErrorTolerance(0.7, 100))
                  .withArcExtension(
                      Point.ofRed(new Pose2d(9.5, 4.4, Rotation2d.kCW_90deg)), -0.0525, -0.51)
                  .withAngularConstraints(
                      Units.rotationsToRadians(2.0), Units.rotationsToRadians(2.0)),
              AutoPoint.ofRed(
                      new Pose2d(
                          10.575, FieldUtil.RED_OUTPOST_BUMP_CENTER.getY(), Rotation2d.kCCW_90deg))
                  .withTransitionTolerance(new PoseErrorTolerance(0.3, 100))
                  .withMarker(Markers.CANCEL_INTAKE_RQ))
          .withLinearConstraints(2.5, 4.0)
          .withAngularConstraints(Units.rotationsToRadians(3.0), Units.rotationsToRadians(3.0))
          .untilFinished(new PoseErrorTolerance(0.3, 100));

  private final AutoSegment driveBackAndShootTwo =
      Trailblazer.segment(
              AutoPoint.ofRed(
                      new Pose2d(
                          10.8, FieldUtil.RED_OUTPOST_BUMP_CENTER.getY(), Rotation2d.kCCW_90deg))
                  .withAngularConstraints(
                      Units.rotationsToRadians(4.0), Units.rotationsToRadians(4.0))
                  .withTransitionTolerance(new PoseErrorTolerance(0.3, 10)),
              AutoPoint.of(
                      () ->
                          bumpCrossingTracker.getPoint(
                              Point.ofRed(
                                  new Pose2d(
                                      13.709,
                                      FieldUtil.RED_OUTPOST_BUMP_CENTER.getY(),
                                      Rotation2d.kCCW_90deg)),
                              Point.ofRed(new Pose2d(13.9, 5.593, Rotation2d.kCCW_90deg))))
                  .withTransitionTolerance(new PoseErrorTolerance(0.3, 100))
                  .withLinearConstraints(4.5, 8),
              AutoPoint.ofRed(
                      new Pose2d(
                          14.0,
                          FieldUtil.RED_OUTPOST_BUMP_CENTER.getY(),
                          Rotation2d.fromDegrees(40)))
                  .withTransitionTolerance(new PoseErrorTolerance(0.5))
                  .withLinearConstraints(4.5, 8)
                  .withMarker(Markers.START_SHOOT_RQ))
          .withLinearConstraints(2.5, 8)
          .withAngularConstraints(Units.rotationsToRadians(2.5), Units.rotationsToRadians(3))
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
          .withLinearConstraints(2.5, 8)
          .withAngularConstraints(Units.rotationsToRadians(1.0), Units.rotationsToRadians(2))
          .untilFinished(new PoseErrorTolerance(0.3, 3));

  public RightIntegratedAuto(RobotManager robotManager, Trailblazer trailblazer) {
    super(IntegratedAutoState.INTAKE_ACROSS_MIDLINE, robotManager, trailblazer);

    this.bumpCrossingTracker = robotManager.localization.imu.bumpCrossingTracker;
  }

  @Override
  public Point getStartingPoint() {
    return Point.ofRed(
        new Pose2d(12.1, FieldUtil.RED_OUTPOST_TRENCH_CENTER.getY(), Rotation2d.k180deg));
  }

  @Override
  protected IntegratedAutoState getNextState(IntegratedAutoState currentState) {
    return switch (currentState) {
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
        if (trailblazer.atGoal(robotManager.localization.getPose()) && timeout(1.0)) {
          yield IntegratedAutoState.INTAKE_BEHIND_HUB;
        } else {
          yield currentState;
        }
      }
      case INTAKE_BEHIND_HUB -> {
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
        if (trailblazer.atGoal(robotManager.localization.getPose()) && timeout(1.0)) {
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
    switch (newState) {
      case INTAKE_ACROSS_MIDLINE -> {
        trailblazer.setActiveSegment(intakeAcrossMidline);
        robotManager.intakeAutoRequest();
        robotManager.powerManager.firstAutoSegmentRequest();
      }
      case DRIVE_BACK_1 -> {
        trailblazer.setActiveSegment(driveBackAndShootOne);
        robotManager.cancelIntakeRequest();
      }
      case SHOOT_1 -> robotManager.prepareScoreRequest();
      case INTAKE_BEHIND_HUB -> {
        trailblazer.setActiveSegment(intakeBehindHub);
        robotManager.intakeAutoRequest();
      }
      case DRIVE_BACK_2 -> {
        trailblazer.setActiveSegment(driveBackAndShootTwo);
      }
      case SHOOT_2 -> robotManager.prepareScoreRequest();
      case DRIVE_BACK_TO_NEUTRAL_ZONE -> trailblazer.setActiveSegment(driveBackToNeutralZone);
      case DONE -> {}
    }
  }

  @Override
  protected void afterTransition(IntegratedAutoState newState) {
    switch (newState) {
      case INTAKE_ACROSS_MIDLINE -> {
        robotManager.homeDeployInAutoRequest();
        robotManager.homeShooterHoodRequest();
      }
      case DRIVE_BACK_1 -> {}
      case SHOOT_1 -> {}
      case INTAKE_BEHIND_HUB -> {
        robotManager.idleRequest();
      }
      case DRIVE_BACK_2 -> {}
      case SHOOT_2 -> {
        robotManager.cancelIntakeRequest();
      }
      case DRIVE_BACK_TO_NEUTRAL_ZONE -> {
        robotManager.idleRequest();
      }
      case DONE -> {
        robotManager.idleRequest();
      }
    }
  }
}
