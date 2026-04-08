package frc.robot.autos.auto_state_machines;

import com.team581.autos.Point;
import com.team581.math.PoseErrorTolerance;
import com.team581.trailblazer.AutoPoint;
import com.team581.trailblazer.Trailblazer;
import com.team581.trailblazer.segments.AutoSegment;
import com.team581.util.FieldUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.autos.BaseImperativeAuto;
import frc.robot.autos.auto_state_machines.auto_state.TestPathAutoState;
import frc.robot.robot_manager.RobotManager;

public class RightTestPathAuto extends BaseImperativeAuto<TestPathAutoState> {

  public enum Markers {
    START_SHOOT_RQ,
    CANCEL_INTAKE_RQ,
    READY_TO_SHOOT_FOR_2
  }

  private final AutoSegment testIntakingWithGlobalConstraints =
      Trailblazer.segment(
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
              AutoPoint.ofRed(new Pose2d(10.575, 5.6, Rotation2d.kCCW_90deg))
                  .withTransitionTolerance(new PoseErrorTolerance(0.5, 100)))
          .withLinearConstraints(3.5, 4.0)
          .withAngularConstraints(Units.rotationsToRadians(3.0), Units.rotationsToRadians(3.0))
          .untilFinished(new PoseErrorTolerance(0.3, 100));

  private final AutoSegment driveBackAndShootOne =
      Trailblazer.segment(
              AutoPoint.ofRed(
                      new Pose2d(
                          13.709, FieldUtil.RED_OUTPOST_BUMP_CENTER.getY(), Rotation2d.kZero))
                  .withTransitionTolerance(new PoseErrorTolerance(0.3, 100))
                  .withLinearConstraints(4.5, 8),
              AutoPoint.ofRed(
                      new Pose2d(
                          14.709, FieldUtil.RED_OUTPOST_BUMP_CENTER.getY(), Rotation2d.kZero))
                  .withTransitionTolerance(new PoseErrorTolerance(0.5))
                  .withLinearConstraints(4.5, 8),
              AutoPoint.ofRed(
                      new Pose2d(
                          13.709, FieldUtil.RED_OUTPOST_TRENCH_CENTER.getY(), Rotation2d.k180deg))
                  .withTransitionTolerance(new PoseErrorTolerance(0.3, 100))
                  .withMarker(Markers.START_SHOOT_RQ),
              AutoPoint.ofRed(
                      new Pose2d(
                          13.0, FieldUtil.RED_OUTPOST_TRENCH_CENTER.getY(), Rotation2d.k180deg))
                  .withTransitionTolerance(new PoseErrorTolerance(0.1, 100)))
          .withLinearConstraints(4.5, 8)
          .withAngularConstraints(Units.rotationsToRadians(1), Units.rotationsToRadians(1))
          .untilFinished(new PoseErrorTolerance(0.3, 3));

  public RightTestPathAuto(RobotManager robotManager, Trailblazer trailblazer) {
    super(TestPathAutoState.INTAKE_ACROSS_MIDLINE, robotManager, trailblazer);
  }

  @Override
  public Point getStartingPoint() {
    return Point.ofRed(
        new Pose2d(12.1, FieldUtil.RED_OUTPOST_TRENCH_CENTER.getY(), Rotation2d.k180deg));
  }

  @Override
  protected TestPathAutoState getNextState(TestPathAutoState currentState) {
    return switch (currentState) {
      case INTAKE_ACROSS_MIDLINE -> {
        if (trailblazer.passedMarker(Markers.CANCEL_INTAKE_RQ)) {
          yield TestPathAutoState.DRIVE_BACK_1;
        } else {
          yield currentState;
        }
      }
      case DRIVE_BACK_1 -> {
        if (trailblazer.passedMarker(Markers.START_SHOOT_RQ)) {
          yield TestPathAutoState.SHOOT_1;
        } else {
          yield currentState;
        }
      }
      case SHOOT_1 -> {
        if (trailblazer.atGoal(robotManager.localization.getPose()) && timeout(6.0)) {
          yield TestPathAutoState.DONE;
        } else {
          yield currentState;
        }
      }
      case DONE -> {
        if (trailblazer.atGoal(robotManager.localization.getPose())) {
          yield TestPathAutoState.DONE;
        } else {
          yield currentState;
        }
      }
      default -> currentState;
    };
  }

  @Override
  protected void whileInState(TestPathAutoState newState) {
    switch (newState) {
      case INTAKE_ACROSS_MIDLINE -> {
        trailblazer.setActiveSegment(testIntakingWithGlobalConstraints);
        robotManager.intakeAutoRequest();
      }
      case DRIVE_BACK_1 -> {
        trailblazer.setActiveSegment(driveBackAndShootOne);
        robotManager.cancelIntakeRequest();
      }
      case SHOOT_1 -> robotManager.prepareScoreRequest();
      case DONE -> {}
    }
  }

  @Override
  protected void afterTransition(TestPathAutoState newState) {
    switch (newState) {
      case INTAKE_ACROSS_MIDLINE -> {
        robotManager.homeDeployInAutoRequest();
        robotManager.homeShooterHoodRequest();
      }
      case DRIVE_BACK_1 -> {}
      case SHOOT_1 -> {}
      case DONE -> {
        robotManager.idleRequest();
      }
    }
  }
}
