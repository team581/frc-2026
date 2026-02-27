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
import frc.robot.autos.auto_state_machines.auto_state.RightPullSwoopShootAutoState;
import frc.robot.climber.ClimbLocation;
import frc.robot.robot_manager.ClimbAssist;
import frc.robot.robot_manager.RobotManager;

public class RightPullSwoopShootAuto extends BaseImperativeAuto<RightPullSwoopShootAutoState> {

  public enum Markers {
    START_SHOOT_RQ
  }

  private final AutoSegment intakeAcrossMidlineOne =
      Trailblazer.segment(
        AutoPoint.ofRed(new Pose2d(11.2, FieldUtil.RED_OUTPOST_TRENCH_CENTER.getY(), Rotation2d.k180deg)),
              AutoPoint.ofRed(
                      new Pose2d(
                          10.235,
                          FieldUtil.RED_OUTPOST_TRENCH_CENTER.getY(),
                          Rotation2d.fromDegrees(-140.0)))
                  .withTransitionTolerance(new PoseErrorTolerance(0.2, 10)),
              AutoPoint.ofRed(new Pose2d(8.85, 7.15, Rotation2d.fromDegrees(-140.0))),
              AutoPoint.ofRed(new Pose2d(7.778, 5.665, Rotation2d.kCW_90deg))
                  .withTransitionTolerance(new PoseErrorTolerance(2, 10)),
              AutoPoint.ofRed(new Pose2d(8.031, 4.41, Rotation2d.fromDegrees(-50.0))),
              AutoPoint.ofRed(new Pose2d(8.859, 3.384, Rotation2d.fromDegrees(-50.0))))
          .withLinearConstraints(3.0, 10)
          .withAngularConstraints(Units.rotationsToRadians(4), Units.rotationsToRadians(4))
          .untilFinished(new PoseErrorTolerance(0.2, 3));

  private final AutoSegment intakeAcrossMidlineTwo =
      Trailblazer.segment(
              AutoPoint.ofRed(
                  new Pose2d(11.2, FieldUtil.RED_OUTPOST_TRENCH_CENTER.getY(), Rotation2d.k180deg)),
              AutoPoint.ofRed(new Pose2d(9.9, 6.865, Rotation2d.kCW_90deg))
                  .withTransitionTolerance(new PoseErrorTolerance(0.2, 100)),
              AutoPoint.of(
                    //  () ->
                          Point.ofRed(
                     //         robotManager
                     //             .clusterMap
                     //             .getBestClusterPose()
                     //             .orElseGet(() -> 
                                  new Pose2d(9.9, 4.5, Rotation2d.kCW_90deg)))
                  .withTransitionTolerance(new PoseErrorTolerance(2, 30)))
          .withLinearConstraints(3.0, 10)
          .untilFinished(new PoseErrorTolerance(0.1, 3));

  public final AutoSegment cleanUpIntakeAndShoot =
      Trailblazer.segment(
              AutoPoint.ofRed(new Pose2d(15.643, 7.45, Rotation2d.kZero)),
              AutoPoint.of(
                      () ->
                          Point.ofRed(
                              robotManager
                                  .clusterMap
                                  .getBestClusterPose()
                                  .orElseGet(
                                      () -> new Pose2d(15.286, 6.074, Rotation2d.kCW_90deg))))
                  .withMarker(Markers.START_SHOOT_RQ)
                  .withTransitionTolerance(new PoseErrorTolerance(0.3, 100)))
          .withLinearConstraints(3.0, 10)
          .withAngularConstraints(Units.rotationsToRadians(4), Units.rotationsToRadians(4))
          .untilFinished(new PoseErrorTolerance(0.1, 3));

  public final AutoSegment driveBackAndFeed =
      Trailblazer.segment(
              AutoPoint.ofRed(new Pose2d(10.379, 3.231, Rotation2d.kCCW_90deg))
                  .withTransitionTolerance(new PoseErrorTolerance(0.5, 20)),
              AutoPoint.ofRed(new Pose2d(10.379, 5.566, Rotation2d.kCCW_90deg)))
          .withLinearConstraints(3.0, 10)
          .withAngularConstraints(Units.rotationsToRadians(4), Units.rotationsToRadians(4))
          .untilFinished(new PoseErrorTolerance(0.3, 3));

  private final AutoSegment driveBackAndShootOne =
      Trailblazer.segment(
              AutoPoint.ofRed(new Pose2d(8.85, 5.8, Rotation2d.kCW_90deg))
                  .withTransitionTolerance(new PoseErrorTolerance(0.5, 3)),
              AutoPoint.ofRed(new Pose2d(10.0, 6.9, Rotation2d.k180deg))
                  .withTransitionTolerance(new PoseErrorTolerance(0.3, 100))
                  .withMarker(Markers.START_SHOOT_RQ),
              AutoPoint.ofRed(
                  new Pose2d(11.3, FieldUtil.RED_OUTPOST_TRENCH_CENTER.getY(), Rotation2d.k180deg)),
              AutoPoint.ofRed(
                  new Pose2d(13.0, FieldUtil.RED_OUTPOST_TRENCH_CENTER.getY(), Rotation2d.k180deg)))
          .withLinearConstraints(3.0, 10)
          .withAngularConstraints(Units.rotationsToRadians(4), Units.rotationsToRadians(4))
          .untilFinished(new PoseErrorTolerance(0.5, 3));

  private final AutoSegment driveBackAndShootTwo =
      Trailblazer.segment(
              AutoPoint.ofRed(new Pose2d(9.9, 5.8, Rotation2d.kCW_90deg))
                  .withTransitionTolerance(new PoseErrorTolerance(0.5, 3)),
              AutoPoint.ofRed(new Pose2d(10.0, 6.9, Rotation2d.k180deg))
                  .withTransitionTolerance(new PoseErrorTolerance(0.3, 100))
                  .withMarker(Markers.START_SHOOT_RQ),
              AutoPoint.ofRed(
                  new Pose2d(11.3, FieldUtil.RED_OUTPOST_TRENCH_CENTER.getY(), Rotation2d.k180deg)),
              AutoPoint.ofRed(
                  new Pose2d(13.5, FieldUtil.RED_OUTPOST_TRENCH_CENTER.getY(), Rotation2d.k180deg)))
          .withLinearConstraints(3.0, 10)
          .withAngularConstraints(Units.rotationsToRadians(4), Units.rotationsToRadians(4))
          .untilFinished(new PoseErrorTolerance(0.5, 3));

  public RightPullSwoopShootAuto(RobotManager robotManager, Trailblazer trailblazer) {
    super(RightPullSwoopShootAutoState.INTAKE_ACROSS_MIDLINE_1, robotManager, trailblazer);
  }

  @Override
  public Point getStartingPoint() {
    return Point.ofRed(
        new Pose2d(12.1, FieldUtil.RED_OUTPOST_TRENCH_CENTER.getY(), Rotation2d.k180deg));
  }

  @Override
  protected RightPullSwoopShootAutoState getNextState(RightPullSwoopShootAutoState currentState) {
    return switch (currentState) {
      case INTAKE_ACROSS_MIDLINE_1 -> {
        if (trailblazer.atGoal(robotManager.localization.getPose())) {
          yield RightPullSwoopShootAutoState.DRIVE_BACK_1;
        } else {
          yield currentState;
        }
      }
      case DRIVE_BACK_1 -> {
        if (trailblazer.passedMarker(Markers.START_SHOOT_RQ)) {
          yield RightPullSwoopShootAutoState.SHOOT_1;
        } else {
          yield currentState;
        }
      }
      case SHOOT_1 -> {
        if (trailblazer.atGoal(robotManager.localization.getPose()) && timeout(3.0)) {
          yield RightPullSwoopShootAutoState.INTAKE_ACROSS_MIDLINE_2;
        } else {
          yield currentState;
        }
      }
      case INTAKE_ACROSS_MIDLINE_2 -> {
        if (trailblazer.atGoal(robotManager.localization.getPose())) {
          yield RightPullSwoopShootAutoState.DRIVE_BACK_2;
        } else {
          yield currentState;
        }
      }
      case DRIVE_BACK_2 -> {
        if (trailblazer.passedMarker(Markers.START_SHOOT_RQ)) {
          yield RightPullSwoopShootAutoState.SHOOT_2;
        } else {
          yield currentState;
        }
      }
      case SHOOT_2 -> {
        if (trailblazer.atGoal(robotManager.localization.getPose())) {
          yield RightPullSwoopShootAutoState.DRIVE_TO_CLIMB;
        } else {
          yield currentState;
        }
      }
      case DRIVE_TO_CLIMB -> {
        if (trailblazer.atGoal(robotManager.localization.getPose())) {
          yield RightPullSwoopShootAutoState.CLIMB;
        } else {
          yield currentState;
        }
      }
      case CLIMB -> {
        if (trailblazer.atGoal(robotManager.localization.getPose())) {
          yield RightPullSwoopShootAutoState.DONE;
        } else {
          yield currentState;
        }
      }
      case DONE -> {
        if (trailblazer.atGoal(robotManager.localization.getPose())) {
          yield RightPullSwoopShootAutoState.DONE;
        } else {
          yield currentState;
        }
      }
      default -> currentState;
    };
  }

  @Override
  protected void whileInState(RightPullSwoopShootAutoState newState) {
    switch (newState) {
      case INTAKE_ACROSS_MIDLINE_1 -> {
        trailblazer.setActiveSegment(intakeAcrossMidlineOne);
        robotManager.intakeRequest();
      }
      case DRIVE_BACK_1 -> {
        trailblazer.setActiveSegment(driveBackAndShootOne);
        robotManager.cancelIntakeRequest();
      }
      case SHOOT_1 -> {
        robotManager.prepareScoreRequest();
      }
      case INTAKE_ACROSS_MIDLINE_2 -> {
        trailblazer.setActiveSegment(intakeAcrossMidlineTwo);
        robotManager.intakeRequest();
      }
      case DRIVE_BACK_2 -> {
        trailblazer.setActiveSegment(driveBackAndShootTwo);
        robotManager.cancelIntakeRequest();
      }
      case SHOOT_2 -> {
        robotManager.prepareScoreRequest();
      }
      case DRIVE_TO_CLIMB -> {
        robotManager.startAutoClimbSequence();
      }
      case CLIMB -> {}
      case DONE -> {}
    }
  }

  @Override
  protected void afterTransition(RightPullSwoopShootAutoState newState) {
    switch (newState) {
      case INTAKE_ACROSS_MIDLINE_1 -> {
        robotManager.homeDeployRequest();
        robotManager.homeShooterHoodRequest();
      }
      case DRIVE_BACK_1 -> {}
      case SHOOT_1 -> {}
      case INTAKE_ACROSS_MIDLINE_2 -> {}
      case DRIVE_BACK_2 -> {}
      case SHOOT_2 -> {}
      case DRIVE_TO_CLIMB -> {
        trailblazer.setActiveSegment(
            ClimbAssist.getLineupClimbAssistSegment(
                robotManager.localization.getPose(), ClimbLocation.CLOSEST));
      }
      case CLIMB -> {}
      case DONE -> {}
    }
  }
}
