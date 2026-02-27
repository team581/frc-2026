package frc.robot.autos.auto_state_machines;

import com.team581.autos.Point;
import com.team581.math.PoseErrorTolerance;
import com.team581.trailblazer.AutoPoint;
import com.team581.trailblazer.Trailblazer;
import com.team581.trailblazer.segments.AutoSegment;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.autos.BaseImperativeAuto;
import frc.robot.autos.auto_state_machines.auto_state.RightStraightShootClimbAutoState;
import frc.robot.climber.ClimbLocation;
import frc.robot.robot_manager.ClimbAssist;
import frc.robot.robot_manager.RobotManager;

public class RightStraightShootClimbAuto
    extends BaseImperativeAuto<RightStraightShootClimbAutoState> {

  public enum Markers {
    START_SHOOT_RQ
  }

  private final AutoSegment intakeAcrossMidlineOne =
      Trailblazer.segment(
              AutoPoint.ofRed(new Pose2d(10.489, 7.45, Rotation2d.k180deg))
                  .withTransitionTolerance(new PoseErrorTolerance(0.7, 30)),
              AutoPoint.ofRed(new Pose2d(9.0, 6.865, Rotation2d.fromDegrees(-126.0))),
              AutoPoint.ofRed(new Pose2d(8.852, 5.4, Rotation2d.kCW_90deg))
                  .withTransitionTolerance(new PoseErrorTolerance(2, 30)))
          .withLinearConstraints(3.0, 10)
          .withAngularConstraints(Units.rotationsToRadians(4), Units.rotationsToRadians(4))
          .untilFinished(new PoseErrorTolerance(0.2, 3));

  private final AutoSegment intakeAcrossMidlineTwo =
      Trailblazer.segment(
              AutoPoint.ofRed(new Pose2d(11.2, 7.45, Rotation2d.k180deg)),
              AutoPoint.ofRed(new Pose2d(9.0, 6.865, Rotation2d.fromDegrees(-126.0))),
              AutoPoint.of(
                      () ->
                          Point.ofRed(
                              robotManager
                                  .clusterMap
                                  .getBestClusterPose()
                                  .orElseGet(() -> new Pose2d(8.852, 5.2, Rotation2d.kCW_90deg))))
                  .withTransitionTolerance(new PoseErrorTolerance(2, 30)))
          .withLinearConstraints(3.0, 10)
          .untilFinished(new PoseErrorTolerance(0.1, 3));

  private final AutoSegment intakeAcrossMidlineThree =
      Trailblazer.segment(
              AutoPoint.ofRed(new Pose2d(11.2, 7.45, Rotation2d.k180deg)),
              AutoPoint.ofRed(new Pose2d(9.0, 6.865, Rotation2d.fromDegrees(-126.0))),
              AutoPoint.of(
                      () ->
                          Point.ofRed(
                              robotManager
                                  .clusterMap
                                  .getBestClusterPose()
                                  .orElseGet(() -> new Pose2d(8.852, 5.0, Rotation2d.kCW_90deg))))
                  .withTransitionTolerance(new PoseErrorTolerance(2, 30)))
          .withLinearConstraints(3.0, 10)
          .untilFinished(new PoseErrorTolerance(0.1, 3));

  private final AutoSegment driveBackAndShootOne =
      Trailblazer.segment(
              AutoPoint.ofRed(new Pose2d(8.85, 5.8, Rotation2d.kCW_90deg))
                  .withTransitionTolerance(new PoseErrorTolerance(0.5, 30)),
              AutoPoint.ofRed(new Pose2d(10.0, 6.9, Rotation2d.k180deg))
                  .withTransitionTolerance(new PoseErrorTolerance(0.3, 100))
                  .withMarker(Markers.START_SHOOT_RQ),
              AutoPoint.ofRed(new Pose2d(11.3, 7.45, Rotation2d.k180deg)),
              AutoPoint.ofRed(new Pose2d(13.0, 7.45, Rotation2d.k180deg))
                  .withLinearConstraints(3.0, 10))
          .untilFinished(new PoseErrorTolerance(0.3, 3));

  private final AutoSegment driveBackAndShootTwo =
      Trailblazer.segment(
              AutoPoint.ofRed(new Pose2d(8.85, 5.8, Rotation2d.kCW_90deg))
                  .withTransitionTolerance(new PoseErrorTolerance(0.5, 30)),
              AutoPoint.ofRed(new Pose2d(10.0, 6.9, Rotation2d.k180deg))
                  .withTransitionTolerance(new PoseErrorTolerance(0.3, 100))
                  .withMarker(Markers.START_SHOOT_RQ),
              AutoPoint.ofRed(new Pose2d(11.3, 7.45, Rotation2d.k180deg)),
              AutoPoint.ofRed(new Pose2d(13.0, 7.45, Rotation2d.k180deg)))
          .withLinearConstraints(3.0, 10)
          .untilFinished(new PoseErrorTolerance(0.5, 3));

  private final AutoSegment driveBackAndShootThree =
      Trailblazer.segment(
              AutoPoint.ofRed(new Pose2d(8.85, 5.8, Rotation2d.kCW_90deg))
                  .withTransitionTolerance(new PoseErrorTolerance(0.5, 30)),
              AutoPoint.ofRed(new Pose2d(10.0, 6.9, Rotation2d.k180deg))
                  .withTransitionTolerance(new PoseErrorTolerance(0.3, 100))
                  .withMarker(Markers.START_SHOOT_RQ),
              AutoPoint.ofRed(new Pose2d(11.3, 7.45, Rotation2d.k180deg)),
              AutoPoint.ofRed(new Pose2d(13.5, 7.45, Rotation2d.k180deg)))
          .withLinearConstraints(3.0, 10)
          .untilFinished(new PoseErrorTolerance(0.5, 3));

  public RightStraightShootClimbAuto(RobotManager robotManager, Trailblazer trailblazer) {
    super(RightStraightShootClimbAutoState.INTAKE_ACROSS_MIDLINE_1, robotManager, trailblazer);
  }

  @Override
  public Point getStartingPoint() {
    return Point.ofRed(new Pose2d(12.1, 7.45, Rotation2d.k180deg));
  }

  @Override
  protected RightStraightShootClimbAutoState getNextState(
      RightStraightShootClimbAutoState currentState) {
    return switch (currentState) {
      case INTAKE_ACROSS_MIDLINE_1 -> {
        if (trailblazer.atGoal(robotManager.localization.getPose())) {
          yield RightStraightShootClimbAutoState.DRIVE_BACK_1;
        } else {
          yield currentState;
        }
      }
      case DRIVE_BACK_1 -> {
        if (trailblazer.passedMarker(Markers.START_SHOOT_RQ)) {
          yield RightStraightShootClimbAutoState.SHOOT_1;
        } else {
          yield currentState;
        }
      }
      case SHOOT_1 -> {
        if (trailblazer.atGoal(robotManager.localization.getPose()) && timeout(5.0)) {
          yield RightStraightShootClimbAutoState.INTAKE_ACROSS_MIDLINE_2;
        } else {
          yield currentState;
        }
      }
      case INTAKE_ACROSS_MIDLINE_2 -> {
        if (trailblazer.atGoal(robotManager.localization.getPose())) {
          yield RightStraightShootClimbAutoState.DRIVE_BACK_2;
        } else {
          yield currentState;
        }
      }
      case DRIVE_BACK_2 -> {
        if (trailblazer.passedMarker(Markers.START_SHOOT_RQ)) {
          yield RightStraightShootClimbAutoState.SHOOT_2;
        } else {
          yield currentState;
        }
      }
      case SHOOT_2 -> {
        if (trailblazer.atGoal(robotManager.localization.getPose()) && timeout(5.0)) {
          yield RightStraightShootClimbAutoState.INTAKE_ACROSS_MIDLINE_3;
        } else {
          yield currentState;
        }
      }
      case INTAKE_ACROSS_MIDLINE_3 -> {
        if (trailblazer.atGoal(robotManager.localization.getPose())) {
          yield RightStraightShootClimbAutoState.DRIVE_BACK_3;
        } else {
          yield currentState;
        }
      }
      case DRIVE_BACK_3 -> {
        if (trailblazer.passedMarker(Markers.START_SHOOT_RQ)) {
          yield RightStraightShootClimbAutoState.SHOOT_3;
        } else {
          yield currentState;
        }
      }
      case SHOOT_3 -> {
        if (trailblazer.atGoal(robotManager.localization.getPose()) && timeout(5.0)) {
          yield RightStraightShootClimbAutoState.DRIVE_TO_CLIMB;
        } else {
          yield currentState;
        }
      }
      case DRIVE_TO_CLIMB -> {
        if (trailblazer.atGoal(robotManager.localization.getPose())) {
          yield RightStraightShootClimbAutoState.CLIMB;
        } else {
          yield currentState;
        }
      }
      case CLIMB -> {
        if (trailblazer.atGoal(robotManager.localization.getPose())) {
          yield RightStraightShootClimbAutoState.DONE;
        } else {
          yield currentState;
        }
      }
      case DONE -> {
        if (trailblazer.atGoal(robotManager.localization.getPose())) {
          yield RightStraightShootClimbAutoState.DONE;
        } else {
          yield currentState;
        }
      }
      default -> currentState;
    };
  }

  @Override
  protected void whileInState(RightStraightShootClimbAutoState newState) {
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
      case INTAKE_ACROSS_MIDLINE_3 -> {
        trailblazer.setActiveSegment(intakeAcrossMidlineThree);
        robotManager.intakeRequest();
      }
      case DRIVE_BACK_3 -> {
        trailblazer.setActiveSegment(driveBackAndShootThree);
        robotManager.cancelIntakeRequest();
      }
      case SHOOT_3 -> {
        robotManager.prepareScoreRequest();
      }
      case DRIVE_TO_CLIMB -> {
        robotManager.startAutoClimbSequence();
        robotManager.prepareScoreRequest();
      }
      case CLIMB -> {}
      case DONE -> {}
    }
  }

  @Override
  protected void afterTransition(RightStraightShootClimbAutoState newState) {
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
      case INTAKE_ACROSS_MIDLINE_3 -> {}
      case DRIVE_BACK_3 -> {}
      case SHOOT_3 -> {}
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
