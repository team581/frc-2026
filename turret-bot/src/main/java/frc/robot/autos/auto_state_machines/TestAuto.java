package frc.robot.autos.auto_state_machines;

import com.team581.autos.Point;
import com.team581.math.PoseErrorTolerance;
import com.team581.trailblazer.AutoPoint;
import com.team581.trailblazer.Trailblazer;
import com.team581.trailblazer.segments.AutoSegment;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.autos.BaseImperativeAuto;
import frc.robot.robot_manager.RobotManager;

/**
 * Test auto that drives around and tests Trailblazer's functionality including constraints, point
 * transitions, and various path segments.
 */
public class TestAuto extends BaseImperativeAuto<TestAutoState> {
  // Test 1: Simple straight movement forward
  private final AutoSegment segment1Straight =
      Trailblazer.segment(
              AutoPoint.ofRed(new Pose2d(0, 0, Rotation2d.kZero)),
              AutoPoint.ofRed(new Pose2d(2, 0, Rotation2d.kZero)))
          .untilFinished(new PoseErrorTolerance(0.05, 3));

  // Test 2: Turn in place and then move forward
  private final AutoSegment segment2Turn =
      Trailblazer.segment(
              AutoPoint.ofRed(new Pose2d(2, 0, Rotation2d.kZero)),
              AutoPoint.ofRed(new Pose2d(2, 0, Rotation2d.kCCW_90deg)),
              AutoPoint.ofRed(new Pose2d(2, 2, Rotation2d.kCCW_90deg)))
          .untilFinished(new PoseErrorTolerance(0.05, 3));

  // Test 3: Movement with linear and angular constraints
  private final AutoSegment segment3Constrained =
      Trailblazer.segment(
              AutoPoint.ofRed(new Pose2d(2, 2, Rotation2d.kCCW_90deg)),
              AutoPoint.ofRed(new Pose2d(4, 2, Rotation2d.kCCW_90deg)))
          // Max velocity 1 m/s, max acceleration 0.5 m/s/s
          .withLinearConstraints(1.0, 0.5)
          // Max angular velocity pi/2 rad/s, max angular acceleration pi/4 rad/s/s
          .withAngularConstraints(Math.PI / 2, Math.PI / 4)
          .untilFinished(new PoseErrorTolerance(0.05, 3));

  // Test 4: Multiple waypoints with point transitions
  private final AutoSegment segment4MultiWaypoint =
      Trailblazer.segment(
              AutoPoint.ofRed(new Pose2d(4, 2, Rotation2d.kCCW_90deg)),
              AutoPoint.ofRed(new Pose2d(4, 3, Rotation2d.kCCW_90deg)),
              AutoPoint.ofRed(new Pose2d(5, 3, Rotation2d.kZero)),
              AutoPoint.ofRed(new Pose2d(5, 4, Rotation2d.kCCW_90deg)))
          .untilFinished(new PoseErrorTolerance(0.05, 3));

  // Test 5: Circular path with rotation
  private final AutoSegment segment5Circle =
      Trailblazer.segment(
              AutoPoint.ofRed(new Pose2d(5, 4, Rotation2d.kCCW_90deg)),
              AutoPoint.ofRed(new Pose2d(6, 5, Rotation2d.fromDegrees(135))),
              AutoPoint.ofRed(new Pose2d(6, 6, Rotation2d.k180deg)),
              AutoPoint.ofRed(new Pose2d(5, 6, Rotation2d.fromDegrees(225))),
              AutoPoint.ofRed(new Pose2d(4, 5, Rotation2d.fromDegrees(270))))
          .untilFinished(new PoseErrorTolerance(0.05, 3));

  public TestAuto(RobotManager robotManager, Trailblazer trailblazer) {
    super(TestAutoState.SEGMENT_1_STRAIGHT, robotManager, trailblazer);
  }

  @Override
  public Point getStartingPoint() {
    return Point.ofRed(Pose2d.kZero);
  }

  @Override
  protected TestAutoState getNextState(TestAutoState currentState) {
    // Transition to next segment when current segment is complete
    if (trailblazer.atGoal(robotManager.localization.getPose())) {
      return switch (currentState) {
        case SEGMENT_1_STRAIGHT -> TestAutoState.SEGMENT_2_TURN;
        case SEGMENT_2_TURN -> TestAutoState.SEGMENT_3_CONSTRAINED;
        case SEGMENT_3_CONSTRAINED -> TestAutoState.SEGMENT_4_MULTI_WAYPOINT;
        case SEGMENT_4_MULTI_WAYPOINT -> TestAutoState.SEGMENT_5_CIRCLE;
        case SEGMENT_5_CIRCLE -> TestAutoState.DONE;
        case DONE -> TestAutoState.DONE;
      };
    }
    return currentState;
  }

  @Override
  protected void whileInState(TestAutoState newState) {
    switch (newState) {
      case SEGMENT_1_STRAIGHT -> robotManager.swerve.trailblazerDriveRequest(segment1Straight);
      case SEGMENT_2_TURN -> robotManager.swerve.trailblazerDriveRequest(segment2Turn);
      case SEGMENT_3_CONSTRAINED ->
          robotManager.swerve.trailblazerDriveRequest(segment3Constrained);
      case SEGMENT_4_MULTI_WAYPOINT ->
          robotManager.swerve.trailblazerDriveRequest(segment4MultiWaypoint);
      case SEGMENT_5_CIRCLE -> robotManager.swerve.trailblazerDriveRequest(segment5Circle);
      case DONE -> {}
    }
  }
}
