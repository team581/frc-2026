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
import frc.robot.deploy.DeployConfig;
import frc.robot.robot_manager.ClimbAssist;
import frc.robot.robot_manager.RobotManager;

public class RightStraightShootClimbAuto
    extends BaseImperativeAuto<RightStraightShootClimbAutoState> {

  private final AutoSegment intakeAcrossMidlineOne =
      Trailblazer.segment(
              AutoPoint.ofRed(new Pose2d(10.489, 7.45, Rotation2d.fromDegrees(-150.0)))
                  .withTransitionTolerance(new PoseErrorTolerance(0.7, 10)),
              AutoPoint.ofRed(new Pose2d(9.0, 6.865, Rotation2d.fromDegrees(-126.0))),
              AutoPoint.ofRed(new Pose2d(8.852, 5.4, Rotation2d.kCW_90deg))
                  .withTransitionTolerance(new PoseErrorTolerance(2, 3)))
          .withLinearConstraints(3.0, 10)
          .withAngularConstraints(Units.rotationsToRadians(4), Units.rotationsToRadians(4))
          .untilFinished(new PoseErrorTolerance(0.2, 3));

  private final AutoSegment intakeAcrossMidlineTwo =
      Trailblazer.segment(
              AutoPoint.ofRed(new Pose2d(11.2, 7.45, Rotation2d.fromDegrees(-150.0))),
              AutoPoint.ofRed(new Pose2d(9.0, 6.865, Rotation2d.fromDegrees(-126.0))),
              AutoPoint.ofRed(new Pose2d(8.852, 5.2, Rotation2d.kCW_90deg))
                  .withTransitionTolerance(new PoseErrorTolerance(2, 3)))
          .withLinearConstraints(3.0, 10)
          .untilFinished(new PoseErrorTolerance(0.1, 3));

  private final AutoSegment driveBack =
      Trailblazer.segment(AutoPoint.ofRed(new Pose2d(8.85, 5.8, Rotation2d.kCW_90deg)))
          .withLinearConstraints(3.0, 10)
          .untilFinished(new PoseErrorTolerance(0.5, 3));

  private final AutoSegment driveToShootOne =
      Trailblazer.segment(
              AutoPoint.ofRed(new Pose2d(10.0, 6.9, Rotation2d.k180deg))
                  .withTransitionTolerance(new PoseErrorTolerance(0.3, 100)),
              AutoPoint.ofRed(new Pose2d(11.3, 7.45, Rotation2d.k180deg)),
              AutoPoint.ofRed(new Pose2d(12.8, 7.45, Rotation2d.k180deg))
                  .withLinearConstraints(3.0, 10))
          .untilFinished(new PoseErrorTolerance(0.3, 3));

  private final AutoSegment driveToShootTwo =
      Trailblazer.segment(
              AutoPoint.ofRed(new Pose2d(10.0, 6.9, Rotation2d.k180deg))
                  .withTransitionTolerance(new PoseErrorTolerance(0.3, 100)),
              AutoPoint.ofRed(new Pose2d(11.3, 7.45, Rotation2d.k180deg)),
              AutoPoint.ofRed(new Pose2d(13.5, 7.45, Rotation2d.k180deg)))
          .withLinearConstraints(3.0, 10)
          .untilFinished(new PoseErrorTolerance(0.5, 3));

  public RightStraightShootClimbAuto(RobotManager robotManager, Trailblazer trailblazer) {
    super(RightStraightShootClimbAutoState.HOMING, robotManager, trailblazer);
  }

  @Override
  public Point getStartingPoint() {
    return Point.ofRed(new Pose2d(13.0, 7.45, Rotation2d.k180deg));
  }

  @Override
  protected RightStraightShootClimbAutoState getNextState(
      RightStraightShootClimbAutoState currentState) {
    if (currentState == RightStraightShootClimbAutoState.HOMING
        && robotManager.deploy.atGoal(DeployConfig.MAX_LENGTH)) {
      return RightStraightShootClimbAutoState.INTAKE_ACROSS_MIDLINE_1;
    }
    if (trailblazer.atGoal(robotManager.localization.getPose())) {
      return switch (currentState) {
        case INTAKE_ACROSS_MIDLINE_1 -> RightStraightShootClimbAutoState.DRIVE_BACK_1;
        case DRIVE_BACK_1 -> RightStraightShootClimbAutoState.SHOOT_1;
        case SHOOT_1 ->
            timeout(7.0) ? RightStraightShootClimbAutoState.INTAKE_ACROSS_MIDLINE_2 : currentState;
        case INTAKE_ACROSS_MIDLINE_2 -> RightStraightShootClimbAutoState.DRIVE_BACK_2;
        case DRIVE_BACK_2 -> RightStraightShootClimbAutoState.SHOOT_2;
        case SHOOT_2 -> RightStraightShootClimbAutoState.DRIVE_TO_CLIMB;
        case DRIVE_TO_CLIMB -> RightStraightShootClimbAutoState.CLIMB;
        case CLIMB -> RightStraightShootClimbAutoState.DONE;
        case DONE -> RightStraightShootClimbAutoState.DONE;
        default -> currentState;
      };
    }
    return currentState;
  }

  @Override
  protected void whileInState(RightStraightShootClimbAutoState newState) {
    switch (newState) {
      case HOMING -> {}
      case INTAKE_ACROSS_MIDLINE_1 -> {
        trailblazer.setActiveSegment(intakeAcrossMidlineOne);
        robotManager.intakeRequest();
      }
      case DRIVE_BACK_1 -> {
        trailblazer.setActiveSegment(driveBack);
        robotManager.cancelIntakeRequest();
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
        robotManager.cancelIntakeRequest();
      }
      case SHOOT_2 -> {
        trailblazer.setActiveSegment(driveToShootTwo);
        robotManager.prepareScoreRequest();
      }
      case DRIVE_TO_CLIMB -> {
        //    robotManager.startAutoClimbSequence();
        robotManager.prepareScoreRequest();
      }
      case CLIMB -> {}
      case DONE -> {}
    }
  }

  @Override
  protected void afterTransition(RightStraightShootClimbAutoState newState) {
    switch (newState) {
      case HOMING -> {
        robotManager.homeDeployRequest();
        robotManager.homeShooterHoodRequest();
      }
      case INTAKE_ACROSS_MIDLINE_1 -> {}
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
