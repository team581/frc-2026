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
import frc.robot.autos.auto_state_machines.auto_state.RightPullSwoopFeedAutoState;
import frc.robot.climber.ClimbLocation;
import frc.robot.robot_manager.ClimbAssist;
import frc.robot.robot_manager.RobotManager;

public class RightPullSwoopFeedAuto extends BaseImperativeAuto<RightPullSwoopFeedAutoState> {

  public enum Markers {
    START_SHOOT_RQ,
    START_FEED_RQ
  }

  private final AutoSegment intakeAcrossMidlineOne =
      Trailblazer.segment(
              AutoPoint.ofRed(new Pose2d(10.235, 7.35, Rotation2d.fromDegrees(-140.0)))
                  .withTransitionTolerance(new PoseErrorTolerance(0.7, 10)),
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
              AutoPoint.ofRed(new Pose2d(8.74, 5.6, Rotation2d.kCW_90deg))
                  .withTransitionTolerance(new PoseErrorTolerance(0.2, 100)),
              AutoPoint.ofRed(new Pose2d(8.74, 4.4, Rotation2d.kCW_90deg)))
          .withLinearConstraints(3.0, 10)
          .withAngularConstraints(Units.rotationsToRadians(4), Units.rotationsToRadians(4))
          .untilFinished(new PoseErrorTolerance(0.1, 3));

  private final AutoSegment cleanUpIntakeAndShoot =
           Trailblazer.segment(
              AutoPoint.ofRed(new Pose2d(15.643, 7.45, Rotation2d.kZero)),
                            AutoPoint.of(
                      () ->
                          Point.ofRed(
                              robotManager
                                  .clusterMap
                                  .getBestClusterPose()
                                  .orElseGet(() -> new Pose2d(15.286, 6.074, Rotation2d.kCW_90deg)))).withMarker(Markers.START_SHOOT_RQ)
                  .withTransitionTolerance(new PoseErrorTolerance(0.3, 100)))
          .withLinearConstraints(3.0, 10)
          .withAngularConstraints(Units.rotationsToRadians(4), Units.rotationsToRadians(4))
          .untilFinished(new PoseErrorTolerance(0.1, 3));

  private final AutoSegment driveBackAndFeed =
      Trailblazer.segment(
              AutoPoint.ofRed(new Pose2d(10.379, 3.231, Rotation2d.kCCW_90deg))
                  .withTransitionTolerance(new PoseErrorTolerance(0.5, 20)),
              AutoPoint.ofRed(new Pose2d(10.379, 5.566, Rotation2d.kCCW_90deg))
                  .withMarker(Markers.START_FEED_RQ))
          .withLinearConstraints(3.0, 10)
          .withAngularConstraints(Units.rotationsToRadians(4), Units.rotationsToRadians(4))
          .untilFinished(new PoseErrorTolerance(0.3, 3));

  private final AutoSegment driveBackAndShoot =
      Trailblazer.segment(
              AutoPoint.ofRed(new Pose2d(8.85, 5.8, Rotation2d.kCW_90deg))
                  .withTransitionTolerance(new PoseErrorTolerance(0.5, 3)),
              AutoPoint.ofRed(new Pose2d(10.0, 6.9, Rotation2d.k180deg))
                  .withTransitionTolerance(new PoseErrorTolerance(0.3, 100))
                  .withMarker(Markers.START_SHOOT_RQ),
              AutoPoint.ofRed(new Pose2d(11.3, 7.45, Rotation2d.kZero)),
              AutoPoint.ofRed(new Pose2d(13.0, 7.45, Rotation2d.kZero)))
          .withLinearConstraints(3.0, 10)
          .withAngularConstraints(Units.rotationsToRadians(4), Units.rotationsToRadians(4))
          .untilFinished(new PoseErrorTolerance(0.5, 3));
        
  public RightPullSwoopFeedAuto(RobotManager robotManager, Trailblazer trailblazer) {
    super(RightPullSwoopFeedAutoState.INTAKE_ACROSS_MIDLINE_1, robotManager, trailblazer);
  }

  @Override
  public Point getStartingPoint() {
    return Point.ofRed(new Pose2d(12.1, 7.45, Rotation2d.k180deg));
  }

  @Override
  protected RightPullSwoopFeedAutoState getNextState(RightPullSwoopFeedAutoState currentState) {
    return switch (currentState) {
      case INTAKE_ACROSS_MIDLINE_1 -> {
        if (trailblazer.atGoal(robotManager.localization.getPose())) {
          yield RightPullSwoopFeedAutoState.DRIVE_BACK_1;
        } else {
          yield currentState;
        }
      }
      case DRIVE_BACK_1 -> {
        if (trailblazer.passedMarker(Markers.START_FEED_RQ)) {
          yield RightPullSwoopFeedAutoState.FEED_1;
        } else {
          yield currentState;
        }
      }
      case FEED_1 -> {
        if (trailblazer.atGoal(robotManager.localization.getPose()) && timeout(3.0)) {
          yield RightPullSwoopFeedAutoState.INTAKE_ACROSS_MIDLINE_2;
        } else {
          yield currentState;
        }
      }
      case INTAKE_ACROSS_MIDLINE_2 -> {
        if (trailblazer.atGoal(robotManager.localization.getPose())) {
          yield RightPullSwoopFeedAutoState.DRIVE_BACK_2;
        } else {
          yield currentState;
        }
      }
      case DRIVE_BACK_2 -> {
        if (trailblazer.passedMarker(Markers.START_SHOOT_RQ)) {
          yield RightPullSwoopFeedAutoState.SHOOT_1;
        } else {
          yield currentState;
        }
      }
      case SHOOT_1 -> {
        if (trailblazer.atGoal(robotManager.localization.getPose()) && timeout(3.0)) {
          yield RightPullSwoopFeedAutoState.CLEAN_UP_INTAKE_1;
        } else {
          yield currentState;
        }
      }
      case CLEAN_UP_INTAKE_1 -> {
        if (trailblazer.passedMarker(Markers.START_SHOOT_RQ)) {
          yield RightPullSwoopFeedAutoState.SHOOT_2;
        } else {
          yield currentState;
        }
      }
      case SHOOT_2 -> {
        if (trailblazer.atGoal(robotManager.localization.getPose())) {
          yield RightPullSwoopFeedAutoState.DRIVE_TO_CLIMB;
        } else {
          yield currentState;
        }
      }
      case DRIVE_TO_CLIMB -> {
        if (trailblazer.atGoal(robotManager.localization.getPose())) {
          yield RightPullSwoopFeedAutoState.CLIMB;
        } else {
          yield currentState;
        }
      }
      case CLIMB -> {
        if (trailblazer.atGoal(robotManager.localization.getPose())) {
          yield RightPullSwoopFeedAutoState.DONE;
        } else {
          yield currentState;
        }
      }
      case DONE -> {
        if (trailblazer.atGoal(robotManager.localization.getPose())) {
          yield RightPullSwoopFeedAutoState.DONE;
        } else {
          yield currentState;
        }
      }
      default -> currentState;
    };
  }

  @Override
  protected void whileInState(RightPullSwoopFeedAutoState newState) {
    switch (newState) {
      case INTAKE_ACROSS_MIDLINE_1 -> {
        trailblazer.setActiveSegment(intakeAcrossMidlineOne);
        robotManager.intakeRequest();
      }
      case DRIVE_BACK_1 -> {
        trailblazer.setActiveSegment(driveBackAndFeed);
        robotManager.cancelIntakeRequest();
      }
      case FEED_1 -> {
        robotManager.prepareFeedRequest();
      }
      case INTAKE_ACROSS_MIDLINE_2 -> {
        trailblazer.setActiveSegment(intakeAcrossMidlineTwo);
        robotManager.intakeRequest();
      }
      case DRIVE_BACK_2 -> {
        trailblazer.setActiveSegment(driveBackAndShoot);
        robotManager.cancelIntakeRequest();
      }
      case SHOOT_1 -> {
        robotManager.prepareScoreRequest();
      }
      case CLEAN_UP_INTAKE_1 -> {
        trailblazer.setActiveSegment(cleanUpIntakeAndShoot);
        robotManager.intakeRequest();
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
  protected void afterTransition(RightPullSwoopFeedAutoState newState) {
    switch (newState) {
      case INTAKE_ACROSS_MIDLINE_1 -> {
        robotManager.homeDeployRequest();
        robotManager.homeShooterHoodRequest();
      }
      case DRIVE_BACK_1 -> {}
      case FEED_1 -> {}
      case INTAKE_ACROSS_MIDLINE_2 -> {}
      case DRIVE_BACK_2 -> {}
      case SHOOT_1 -> {}
      case CLEAN_UP_INTAKE_1 -> {}
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
