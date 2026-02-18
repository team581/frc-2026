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
import frc.robot.autos.auto_state_machines.auto_state.RightSwoopShootClimbAutoState;
import frc.robot.climber.ClimbLocation;
import frc.robot.robot_manager.ClimbAssist;
import frc.robot.robot_manager.RobotManager;

@SuppressWarnings("unused")
public class RightSwoopShootClimbAuto extends BaseImperativeAuto<RightSwoopShootClimbAutoState> {

  public enum Markers {
    START_SHOOT_RQ,
    START_INTAKE_RQ,
  }

  private final AutoSegment intakeAcrossMidlineOne =
      Trailblazer.segment(
              AutoPoint.ofRed(new Pose2d(10.358, 7.4, Rotation2d.fromDegrees(-163.0))),
              AutoPoint.ofRed(new Pose2d(8.859, 7.15, Rotation2d.fromDegrees(-140.0))),
              AutoPoint.ofRed(new Pose2d(7.778, 5.665, Rotation2d.kCW_90deg))
                  .withTransitionTolerance(new PoseErrorTolerance(1, 100)),
              AutoPoint.ofRed(new Pose2d(7.778, 5.665, Rotation2d.kZero)),
              AutoPoint.ofRed(new Pose2d(9.227, 4.549, Rotation2d.kZero)))
          .withLinearConstraints(3.5, 10)
          .withAngularConstraints(Units.rotationsToRadians(4), Units.rotationsToRadians(4))
          .untilFinished(new PoseErrorTolerance(0.3, 3));

  private final AutoSegment driveBackAndShoot =
      Trailblazer.segment(
              AutoPoint.ofRed(new Pose2d(9.505, 5.933, Rotation2d.kCW_90deg)),
              AutoPoint.ofRed(new Pose2d(10.293, 7.5, Rotation2d.k180deg))
                  .withMarker(Markers.START_SHOOT_RQ),
              AutoPoint.ofRed(new Pose2d(12.787, 7.5, Rotation2d.k180deg)))
          .withLinearConstraints(3.5, 10)
          .withAngularConstraints(Units.rotationsToDegrees(4), Units.rotationsToDegrees(4))
          .untilFinished(new PoseErrorTolerance(0.3, 3));

  public RightSwoopShootClimbAuto(RobotManager robotManager, Trailblazer trailblazer) {
    super(RightSwoopShootClimbAutoState.INTAKE_ACROSS_MIDLINE_1, robotManager, trailblazer);
  }

  @Override
  public Point getStartingPoint() {
    return Point.ofRed(new Pose2d(12.15, 7.55, Rotation2d.k180deg));
  }

  @Override
  protected RightSwoopShootClimbAutoState getNextState(RightSwoopShootClimbAutoState currentState) {
    return switch (currentState) {
      case INTAKE_ACROSS_MIDLINE_1 -> {
        if (trailblazer.atGoal(robotManager.localization.getPose())) {
          yield RightSwoopShootClimbAutoState.DRIVE_BACK_1;
        } else {
          yield currentState;
        }
      }
      case DRIVE_BACK_1 -> {
        if (trailblazer.passedMarker(Markers.START_SHOOT_RQ)) {
          yield RightSwoopShootClimbAutoState.SHOOT_1;
        } else {
          yield currentState;
        }
      }
      case SHOOT_1 -> {
        if (trailblazer.atGoal(robotManager.localization.getPose()) && timeout(3.0)) {
          yield RightSwoopShootClimbAutoState.INTAKE_ACROSS_MIDLINE_2;
        } else {
          yield currentState;
        }
      }
      case INTAKE_ACROSS_MIDLINE_2 -> {
        if (trailblazer.atGoal(robotManager.localization.getPose())) {
          yield RightSwoopShootClimbAutoState.DRIVE_BACK_2;
        } else {
          yield currentState;
        }
      }
      case DRIVE_BACK_2 -> {
        if (trailblazer.passedMarker(Markers.START_SHOOT_RQ)) {
          yield RightSwoopShootClimbAutoState.SHOOT_2;
        } else {
          yield currentState;
        }
      }
      case SHOOT_2 -> {
        if (trailblazer.atGoal(robotManager.localization.getPose())) {
          yield RightSwoopShootClimbAutoState.DRIVE_TO_CLIMB;
        } else {
          yield currentState;
        }
      }
      case DRIVE_TO_CLIMB -> {
        if (trailblazer.atGoal(robotManager.localization.getPose())) {
          yield RightSwoopShootClimbAutoState.CLIMB;
        } else {
          yield currentState;
        }
      }
      case CLIMB -> {
        if (trailblazer.atGoal(robotManager.localization.getPose())) {
          yield RightSwoopShootClimbAutoState.DONE;
        } else {
          yield currentState;
        }
      }
      case DONE -> {
        if (trailblazer.atGoal(robotManager.localization.getPose())) {
          yield RightSwoopShootClimbAutoState.DONE;
        } else {
          yield currentState;
        }
      }
      default -> currentState;
    };
  }

  @Override
  protected void whileInState(RightSwoopShootClimbAutoState newState) {
    switch (newState) {
      case INTAKE_ACROSS_MIDLINE_1 -> {
        trailblazer.setActiveSegment(intakeAcrossMidlineOne);
        robotManager.intakeRequest();
      }
      case DRIVE_BACK_1 -> {
        trailblazer.setActiveSegment(driveBackAndShoot);
        robotManager.cancelIntakeRequest();
      }
      case SHOOT_1 -> {
        robotManager.prepareScoreRequest();
      }
      case INTAKE_ACROSS_MIDLINE_2 -> {
        trailblazer.setActiveSegment(intakeAcrossMidlineOne);
        robotManager.intakeRequest();
      }
      case DRIVE_BACK_2 -> {
        trailblazer.setActiveSegment(driveBackAndShoot);
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
  protected void afterTransition(RightSwoopShootClimbAutoState newState) {
    switch (newState) {
      case INTAKE_ACROSS_MIDLINE_1 -> {
        robotManager.homeDeployRequest();
        robotManager.homeShooterHoodRequest();
      }
      case DRIVE_BACK_1 -> {}
      case SHOOT_1 -> {}
      case INTAKE_ACROSS_MIDLINE_2 -> {
        robotManager.idleRequest();
      }
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
