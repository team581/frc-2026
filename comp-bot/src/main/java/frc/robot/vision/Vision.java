package frc.robot.vision;

import com.team581.util.state_machines.StateMachineSubsystem;
import com.team581.vision.results.OptionalTagResult;
import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.imu.Imu;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.vision.limelight.Limelight;
import frc.robot.vision.limelight.LimelightState;

public class Vision extends StateMachineSubsystem<VisionState> {
  private final Debouncer seeingHubTagDebouncer = new Debouncer(0.5, DebounceType.kFalling);

  private final Imu imu;
  private final Limelight shooterLimelight;
  private final Limelight leftLimelight;
  private final Limelight rightLimelight;

  private final Limelight groundLimelight;

  private OptionalTagResult shooterResult = new OptionalTagResult();
  private OptionalTagResult leftResult = new OptionalTagResult();
  private OptionalTagResult rightResult = new OptionalTagResult();

  private double robotHeading;

  private double robotAngularVelocity;

  private boolean hasSeenTag = false;
  private boolean seeingTag = false;

  private boolean seeingHubTags = false;

  public Vision(
      Imu imu,
      Limelight shooterLimelight,
      Limelight leftLimelight,
      Limelight rightLimelight,
      Limelight groundLimelight) {
    super(SubsystemPriority.VISION, VisionState.TAGS);
    this.imu = imu;
    this.shooterLimelight = shooterLimelight;
    this.leftLimelight = leftLimelight;
    this.rightLimelight = rightLimelight;
    this.groundLimelight = groundLimelight;
  }

  @Override
  protected VisionState getNextState(VisionState currentState) {
    return switch (currentState) {
      case HUB_TAGS, WAITING_FOR_HUB_TAGS -> {
        if (seeingHubTags) {
          yield VisionState.HUB_TAGS;
        }
        yield VisionState.WAITING_FOR_HUB_TAGS;
      }
      default -> currentState;
    };
  }

  @Override
  protected void collectInputs() {
    robotAngularVelocity = imu.getRobotAngularVelocity();

    shooterResult = shooterLimelight.getTagResult();
    leftResult = leftLimelight.getTagResult();
    rightResult = rightLimelight.getTagResult();

    if (shooterResult.isPresent() || leftResult.isPresent() || rightResult.isPresent()) {
      hasSeenTag = true;
      seeingTag = true;
    } else {
      seeingTag = false;
    }

    seeingHubTags =
        seeingHubTagDebouncer.calculate(
            shooterLimelight.seeingHubTag()
                || leftLimelight.seeingHubTag()
                || rightLimelight.seeingHubTag());
  }

  public void setEstimatedPoseAngle(double robotHeading) {
    this.robotHeading = robotHeading;
  }

  public OptionalTagResult getShooterLimelightTagResult() {
    return shooterResult;
  }

  public OptionalTagResult getLeftLimelightTagResult() {
    return leftResult;
  }

  public OptionalTagResult getRightLimelightTagResult() {
    return rightResult;
  }

  public boolean seeingTag() {
    return seeingTag || RobotBase.isSimulation();
  }

  public boolean hasSeenTag() {
    return hasSeenTag;
  }

  public void setState(VisionState state) {
    if (state == VisionState.HUB_TAGS && getState() == VisionState.WAITING_FOR_HUB_TAGS) {
      return;
    }
    if (state == VisionState.HUB_TAGS) {
      state = VisionState.WAITING_FOR_HUB_TAGS;
    }

    setStateFromRequest(state);
  }

  @Override
  protected void afterTransition(VisionState newState) {
    switch (newState) {
      case TAGS -> {
        shooterLimelight.setState(LimelightState.TAGS);
        leftLimelight.setState(LimelightState.TAGS);
        rightLimelight.setState(LimelightState.TAGS);

        groundLimelight.setState(LimelightState.CLUSTER_MAP);
      }
      case HUB_TAGS -> {
        shooterLimelight.setState(LimelightState.HUB_TAGS);
        leftLimelight.setState(LimelightState.HUB_TAGS);
        rightLimelight.setState(LimelightState.HUB_TAGS);
        groundLimelight.setState(LimelightState.CLUSTER_MAP);
      }
      case WAITING_FOR_HUB_TAGS -> {
        shooterLimelight.setState(LimelightState.TAGS);
        leftLimelight.setState(LimelightState.TAGS);
        rightLimelight.setState(LimelightState.TAGS);
        groundLimelight.setState(LimelightState.CLUSTER_MAP);
      }
      default -> {}
    }
  }

  @Override
  public void whileInState(VisionState currentState) {
    // Send IMU data to all limelights
    shooterLimelight.sendImuData(robotHeading, robotAngularVelocity, 0.0, 0.0, 0.0, 0.0);
    leftLimelight.sendImuData(robotHeading, robotAngularVelocity, 0.0, 0.0, 0.0, 0.0);
    rightLimelight.sendImuData(robotHeading, robotAngularVelocity, 0.0, 0.0, 0.0, 0.0);

    DogLog.log("Vision/SeeingTag", seeingTag);
  }
}
