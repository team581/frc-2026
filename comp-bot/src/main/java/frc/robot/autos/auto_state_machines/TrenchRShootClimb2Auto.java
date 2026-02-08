package frc.robot.autos.auto_state_machines;

import com.team581.autos.Point;
import com.team581.math.PoseErrorTolerance;
import com.team581.trailblazer.AutoPoint;
import com.team581.trailblazer.Trailblazer;
import com.team581.trailblazer.segments.AutoSegment;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.autos.BaseImperativeAuto;
import frc.robot.autos.auto_state_machines.auto_state.TrenchRShootClimb2AutoState;
import frc.robot.climber.ClimbLocation;
import frc.robot.robot_manager.ClimbAssist;
import frc.robot.robot_manager.RobotManager;

public class TrenchRShootClimb2Auto extends BaseImperativeAuto<TrenchRShootClimb2AutoState> {

  private final AutoSegment intakeAcrossMidlineOne =
      Trailblazer.segment(
              AutoPoint.ofRed(new Pose2d(12.0, 7.55, Rotation2d.kCW_90deg)),
              AutoPoint.ofRed(new Pose2d(10.489, 7.55, Rotation2d.kCW_90deg)),
              AutoPoint.ofRed(new Pose2d(8.852, 5.442, Rotation2d.kCW_90deg)))
          .withLinearConstraints(3, 3)
          .untilFinished(new PoseErrorTolerance(0.5, 3));

  private final AutoSegment intakeAcrossMidlineTwo =
      Trailblazer.segment(
              AutoPoint.ofRed(new Pose2d(12.0, 7.55, Rotation2d.kCW_90deg)),
              AutoPoint.ofRed(new Pose2d(10.489, 7.55, Rotation2d.kCW_90deg)),
              AutoPoint.ofRed(new Pose2d(8.852, 4.9, Rotation2d.kCW_90deg)))
          .withLinearConstraints(3, 3)
          .untilFinished(new PoseErrorTolerance(0.5, 3));

  private final AutoSegment driveBack =
      Trailblazer.segment(
              AutoPoint.ofRed(new Pose2d(8.852, 5.8, Rotation2d.kCW_90deg)),
              AutoPoint.ofRed(new Pose2d(10.489, 7.55, Rotation2d.kCW_90deg)),
              AutoPoint.ofRed(new Pose2d(12.0, 7.55, Rotation2d.kCW_90deg)))
          .withLinearConstraints(3, 3)
          .untilFinished(new PoseErrorTolerance(0.5, 3));
  private final AutoSegment driveToShootOne =
      Trailblazer.segment(
              AutoPoint.ofRed(new Pose2d(12.3, 7.55, Rotation2d.kCW_90deg)),
              AutoPoint.ofRed(new Pose2d(13.0, 7.55, Rotation2d.kCW_90deg))
                  .withLinearConstraints(3, 3))
          .untilFinished(new PoseErrorTolerance(0.1, 3));

  private final AutoSegment driveToShootTwo =
      Trailblazer.segment(
              AutoPoint.ofRed(new Pose2d(12.3, 7.55, Rotation2d.kCW_90deg)),
              AutoPoint.ofRed(new Pose2d(13.24, 7.55, Rotation2d.kCW_90deg)),
              AutoPoint.ofRed(new Pose2d(13.9, 5.92, Rotation2d.kCW_90deg)))
          .withLinearConstraints(3, 3)
          .untilFinished(new PoseErrorTolerance(0.5, 3));

  public TrenchRShootClimb2Auto(RobotManager robotManager, Trailblazer trailblazer) {
    super(TrenchRShootClimb2AutoState.INTAKE_ACROSS_MIDLINE_1, robotManager, trailblazer);
  }

  @Override
  public Point getStartingPoint() {
    return Point.ofRed(new Pose2d(12.0, 7.55, Rotation2d.kCW_90deg));
  }

  @Override
  protected TrenchRShootClimb2AutoState getNextState(TrenchRShootClimb2AutoState currentState) {
    if (trailblazer.atGoal(robotManager.localization.getPose())) {
      return switch (currentState) {
        case INTAKE_ACROSS_MIDLINE_1 -> TrenchRShootClimb2AutoState.DRIVE_BACK_1;
        case DRIVE_BACK_1 -> TrenchRShootClimb2AutoState.SHOOT_1;
        case SHOOT_1 ->
            timeout(3.0) ? TrenchRShootClimb2AutoState.INTAKE_ACROSS_MIDLINE_2 : currentState;
        case INTAKE_ACROSS_MIDLINE_2 -> TrenchRShootClimb2AutoState.DRIVE_BACK_2;
        case DRIVE_BACK_2 -> TrenchRShootClimb2AutoState.SHOOT_2;
        case SHOOT_2 -> TrenchRShootClimb2AutoState.DRIVE_TO_CLIMB;
        case DRIVE_TO_CLIMB -> TrenchRShootClimb2AutoState.CLIMB;
        case CLIMB -> TrenchRShootClimb2AutoState.DONE;
        case DONE -> TrenchRShootClimb2AutoState.DONE;
      };
    }
    return currentState;
  }

  @Override
  protected void whileInState(TrenchRShootClimb2AutoState newState) {
    switch (newState) {
      case INTAKE_ACROSS_MIDLINE_1 -> {
        trailblazer.setActiveSegment(intakeAcrossMidlineOne);
        robotManager.intakeRequest();
      }
      case DRIVE_BACK_1 -> {
        trailblazer.setActiveSegment(driveBack);
        robotManager.idleRequest();
      }
      case SHOOT_1 -> {
        trailblazer.setActiveSegment(driveToShootOne);
        robotManager.prepareScoreRequest();
      }
      case INTAKE_ACROSS_MIDLINE_2 -> {
        trailblazer.setActiveSegment(intakeAcrossMidlineTwo);
        robotManager.intakeRequest();
      }
      case DRIVE_BACK_2 -> {
        trailblazer.setActiveSegment(driveBack);
        robotManager.idleRequest();
      }
      case SHOOT_2 -> {
        trailblazer.setActiveSegment(driveToShootTwo);
        robotManager.prepareScoreRequest();
      }
      case DRIVE_TO_CLIMB -> {
        trailblazer.setActiveSegment(
            ClimbAssist.getClimbAssistSegment(
                robotManager.localization.getPose(), ClimbLocation.CLOSEST));
        robotManager.startAutoClimbSequence();
      }
      case CLIMB -> {}
      case DONE -> {}
    }
  }

  @Override
  protected void afterTransition(TrenchRShootClimb2AutoState newState) {
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
      case DRIVE_TO_CLIMB -> {}
      case CLIMB -> {}
      case DONE -> {}
    }
  }
}
