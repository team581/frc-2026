package frc.robot.autos.auto_state_machines;

import com.team581.autos.Point;
import com.team581.math.PoseErrorTolerance;
import com.team581.trailblazer.AutoPoint;
import com.team581.trailblazer.Trailblazer;
import com.team581.trailblazer.segments.AutoSegment;
import com.team581.util.FieldUtil;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import frc.robot.autos.BaseImperativeAuto;
import frc.robot.autos.auto_state_machines.auto_states.IntegrationTestState;
import frc.robot.config.FeatureFlags;
import frc.robot.robot_manager.RobotManager;

public class IntegrationTest extends BaseImperativeAuto<IntegrationTestState> {

  private IntegrationTestState beforePauseState = IntegrationTestState.PAUSED;
  private static final double MAX_VELOCITY = 1.0;
  private static final double MAX_ACCELERATION = 1.0;

  private boolean aButtonReleased = true;
  private boolean bButtonReleased = true;
  private boolean xButtonReleased = true;

  private static final Pose2d RED_START_POSE =
      FieldUtil.HUB_POSE
          .redPose()
          .plus(new Transform2d(Units.inchesToMeters(60.0), 0.0, Rotation2d.kZero));

  private final AutoSegment segment1DriveToStart =
      Trailblazer.segment(AutoPoint.ofRed(RED_START_POSE))
          .withLinearConstraints(MAX_VELOCITY, MAX_ACCELERATION)
          .untilFinished(new PoseErrorTolerance(0.05, 3));

  private final AutoSegment segment2CloseCenteredWithHub =
      Trailblazer.segment(
              AutoPoint.ofRed(RED_START_POSE.plus(new Transform2d(-0.25, 0.0, Rotation2d.kZero))))
          .withLinearConstraints(MAX_VELOCITY, MAX_ACCELERATION)
          .untilFinished(new PoseErrorTolerance(0.05, 3));

  private final AutoSegment segment3BackCenteredWithHub =
      Trailblazer.segment(
              // Climb area
              AutoPoint.ofRed(new Pose2d(14.929, RED_START_POSE.getY(), Rotation2d.kZero)))
          .withLinearConstraints(MAX_VELOCITY, MAX_ACCELERATION)
          .untilFinished(new PoseErrorTolerance(0.05, 3));

  private final AutoSegment segment4RightTrench =
      Trailblazer.segment(
              AutoPoint.ofRed(new Pose2d(14.558, 6.399, Rotation2d.fromDegrees(110))),
              AutoPoint.ofRed(new Pose2d(13.36, 7.29, Rotation2d.fromDegrees(-90))))
          .withLinearConstraints(MAX_VELOCITY, MAX_ACCELERATION)
          .untilFinished(new PoseErrorTolerance(0.05, 3));

  private final AutoSegment segment5RightTrench =
      Trailblazer.segment(
              // Climb area
              AutoPoint.ofRed(new Pose2d(14.929, RED_START_POSE.getY(), Rotation2d.k180deg)),
              AutoPoint.ofRed(new Pose2d(13.36, 2.54, Rotation2d.fromDegrees(-45))))
          .withLinearConstraints(MAX_VELOCITY, MAX_ACCELERATION)
          .untilFinished(new PoseErrorTolerance(0.05, 3));

  public IntegrationTest(RobotManager robotManager, Trailblazer trailblazer) {
    super(IntegrationTestState.SEGMENT_1_DRIVE_TO_START, robotManager, trailblazer);
  }

  public void pauseRequest() {
    if (getState() == IntegrationTestState.PAUSED) {
      DogLog.timestamp("IntegrationTest/UndoPause");
      setStateFromRequest(beforePauseState);
    }

    var state = getState();
    beforePauseState = state;
    DogLog.log("IntegrationTest/GetStateBeforeSet", state);
    setStateFromRequest(IntegrationTestState.PAUSED);
  }

  public void skipRequest() {
    setStateFromRequest(getState().nextState());
  }

  public void previousRequest() {
    setStateFromRequest(getState().previousState());
  }

  @Override
  public Point getStartingPoint() {
    return Point.ofRed(Pose2d.kZero);
  }

  @Override
  protected IntegrationTestState getNextState(IntegrationTestState currentState) {
    if (trailblazer.atGoal(robotManager.localization.getPose().getTranslation())) {
      return switch (currentState) {
        case SEGMENT_2_CLOSE_CENTERED_WITH_HUB ->
            timeout(2.0) ? IntegrationTestState.SEGMENT_3_BACK_CENTERED_WITH_HUB : currentState;
        case PAUSED -> currentState;
        default -> currentState.nextState();
      };
    }
    return currentState;
  }

  @Override
  protected void whileInState(IntegrationTestState newState) {
    DogLog.log("IntegrationTest/BeforePauseState", beforePauseState);
    if (FeatureFlags.INTEGRATION_TEST.getAsBoolean()) {
      if (robotManager.hardware.driverController.getAButton()) {
        if (aButtonReleased) {
          DogLog.timestamp("IntegrationTest/AButton");
          aButtonReleased = false;
          pauseRequest();
        }
      } else {
        aButtonReleased = true;
      }
      if (robotManager.hardware.driverController.getBButton()) {
        if (bButtonReleased) {
          DogLog.timestamp("IntegrationTest/BButton");

          bButtonReleased = false;
          skipRequest();
        }
      } else {
        bButtonReleased = true;
      }
      if (robotManager.hardware.driverController.getXButton()) {
        if (xButtonReleased) {
          DogLog.timestamp("IntegrationTest/XButton");

          xButtonReleased = false;
          previousRequest();
        }
      } else {
        xButtonReleased = true;
      }
    }
    switch (newState) {
      case SEGMENT_1_DRIVE_TO_START -> {
        trailblazer.setActiveSegment(segment1DriveToStart);
        robotManager.idleRequest();
      }
      case SEGMENT_2_CLOSE_CENTERED_WITH_HUB -> {
        trailblazer.setActiveSegment(segment2CloseCenteredWithHub);
        robotManager.scoreRequest();
      }
      case SEGMENT_3_BACK_CENTERED_WITH_HUB -> {
        trailblazer.setActiveSegment(segment3BackCenteredWithHub);
        robotManager.scoreRequest();
      }
      case SEGMENT_4_RIGHT_TRENCH -> {
        trailblazer.setActiveSegment(segment4RightTrench);
        robotManager.scoreRequest();
      }
      case SEGMENT_5_LEFT_RAMP -> {
        trailblazer.setActiveSegment(segment5RightTrench);
        robotManager.scoreRequest();
      }
      case PAUSED -> {
        robotManager.idleRequest();
      }
    }
  }

  @Override
  public boolean shouldRun() {
    return FeatureFlags.INTEGRATION_TEST.getAsBoolean();
  }
}
