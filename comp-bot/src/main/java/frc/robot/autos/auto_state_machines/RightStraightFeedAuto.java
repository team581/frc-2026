package frc.robot.autos.auto_state_machines;

import com.team581.autos.Point;
import com.team581.math.PoseErrorTolerance;
import com.team581.trailblazer.AutoPoint;
import com.team581.trailblazer.Trailblazer;
import com.team581.trailblazer.segments.AutoSegment;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.autos.BaseImperativeAuto;
import frc.robot.autos.auto_state_machines.auto_state.RightStraightFeedAutoState;
import frc.robot.robot_manager.RobotManager;

@SuppressWarnings("unused")
public class RightStraightFeedAuto extends BaseImperativeAuto<RightStraightFeedAutoState> {

  private final AutoSegment segment1DriveToMidline =
      // TODO: update poses

      Trailblazer.segment(
              AutoPoint.ofRed(new Pose2d(14.0, 5.9, Rotation2d.kZero)),
              AutoPoint.ofRed(new Pose2d(11.912, 5.9, Rotation2d.kZero)),
              AutoPoint.ofRed(new Pose2d(10.218, 5.9, Rotation2d.kZero)))
          .untilFinished(new PoseErrorTolerance(0.05, 3));

  private final AutoSegment intakeAcrossMidline =
      // TODO: update poses

      Trailblazer.segment(
              AutoPoint.ofRed(new Pose2d(9.32, 5.8, Rotation2d.kCW_90deg)),
              AutoPoint.ofRed(new Pose2d(9.32, 4.341, Rotation2d.kCW_90deg)),
              AutoPoint.ofRed(new Pose2d(9.32, 2.885, Rotation2d.kCW_90deg)))
          .withLinearConstraints(3, 3)
          .untilFinished(new PoseErrorTolerance(0.05, 3));

  private final AutoSegment segment2DriveToFeed =
      // TODO: update poses

      Trailblazer.segment(
              AutoPoint.ofRed(new Pose2d(9.32, 2.885, Rotation2d.kCW_90deg)),
              AutoPoint.ofRed(new Pose2d(9.32, 4.341, Rotation2d.kCW_90deg)),
              AutoPoint.ofRed(new Pose2d(9.32, 5.6, Rotation2d.k180deg)),
              AutoPoint.ofRed(new Pose2d(11.912, 5.6, Rotation2d.k180deg)),
              AutoPoint.ofRed(new Pose2d(12.9, 5.6, Rotation2d.k180deg)))
          .withLinearConstraints(3, 3)
          .untilFinished(new PoseErrorTolerance(0.05, 3));

  private final AutoSegment segment3DriveBack =
      // TODO: update poses
      Trailblazer.segment(
              AutoPoint.ofRed(new Pose2d(13.751, 5.6, Rotation2d.fromDegrees(-134.0))),
              AutoPoint.ofRed(new Pose2d(14.305, 4.797, Rotation2d.fromDegrees(-164.0))))
          .untilFinished(new PoseErrorTolerance(0.05, 3));

  private final AutoSegment segment4IntakeAllianceSide =
      // TODO: update poses
      Trailblazer.segment(AutoPoint.ofRed(Pose2d.kZero))
          .untilFinished(new PoseErrorTolerance(0, null));

  private final AutoSegment segment5DriveToShoot =
      // TODO: update poses
      Trailblazer.segment(AutoPoint.ofRed(Pose2d.kZero))
          .untilFinished(new PoseErrorTolerance(0, null));

  public RightStraightFeedAuto(RobotManager robotManager, Trailblazer trailblazer) {
    super(RightStraightFeedAutoState.SEGMENT_1_DRIVE_TO_MIDLINE, robotManager, trailblazer);
  }

  @Override
  public Point getStartingPoint() {
    // TODO: update pose
    return Point.ofRed(new Pose2d(14.0, 6.0, Rotation2d.kZero));
  }

  @Override
  protected RightStraightFeedAutoState getNextState(RightStraightFeedAutoState currentState) {
    if (trailblazer.atGoal(robotManager.localization.getPose())) {
      return switch (currentState) {
        case SEGMENT_1_DRIVE_TO_MIDLINE -> RightStraightFeedAutoState.INTAKE_ACROSS_MIDLINE_1;
        case INTAKE_ACROSS_MIDLINE_1 -> RightStraightFeedAutoState.SEGMENT_2_DRIVE_TO_FEED_1;
        case SEGMENT_2_DRIVE_TO_FEED_1 -> RightStraightFeedAutoState.FEED_1;
        case FEED_1 -> RightStraightFeedAutoState.INTAKE_ACROSS_MIDLINE_2;
        case INTAKE_ACROSS_MIDLINE_2 -> RightStraightFeedAutoState.SEGMENT_3_DRIVE_TO_FEED_2;
        case SEGMENT_3_DRIVE_TO_FEED_2 -> RightStraightFeedAutoState.FEED_2;
        case FEED_2 -> RightStraightFeedAutoState.SEGMENT_4_DRIVE_BACK;
        case SEGMENT_4_DRIVE_BACK -> RightStraightFeedAutoState.SEGMENT_5_INTAKE_ALLIANCE_SIDE;
        case SEGMENT_5_INTAKE_ALLIANCE_SIDE -> RightStraightFeedAutoState.SEGMENT_6_DRIVE_TO_SHOOT;
        case SEGMENT_6_DRIVE_TO_SHOOT -> RightStraightFeedAutoState.SHOOT;
        case SHOOT -> RightStraightFeedAutoState.DONE;
        case DONE -> RightStraightFeedAutoState.DONE;
      };
    }
    return currentState;
  }

  @Override
  protected void whileInState(RightStraightFeedAutoState newState) {
    switch (newState) {
      // TODO: update state actions
      case SEGMENT_1_DRIVE_TO_MIDLINE -> {}
      case INTAKE_ACROSS_MIDLINE_1 -> {}
      case INTAKE_ACROSS_MIDLINE_2 -> {}
      case SEGMENT_2_DRIVE_TO_FEED_1 -> {}
      case SEGMENT_3_DRIVE_TO_FEED_2 -> {}
      case FEED_1 -> {}
      case FEED_2 -> {}
      case SEGMENT_4_DRIVE_BACK -> {}
      case SEGMENT_5_INTAKE_ALLIANCE_SIDE -> {}
      case SEGMENT_6_DRIVE_TO_SHOOT -> {}
      case SHOOT -> {}
      case DONE -> {}
    }
  }
}
