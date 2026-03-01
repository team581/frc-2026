package frc.robot.vision;

import com.team581.math.MathHelpers;
import com.team581.util.state_machines.StateMachineSubsystem;
import com.team581.vision.results.OptionalTagResult;
import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.imu.Imu;
import frc.robot.turret.TurretConfig;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.vision.limelight.Limelight;
import frc.robot.vision.limelight.LimelightState;
import java.util.Optional;
import java.util.OptionalDouble;

public class Vision extends StateMachineSubsystem<VisionState> {
  private final Debouncer seeingTagDebouncer = new Debouncer(1.0, DebounceType.kFalling);
  private final Debouncer seeingHubTagDebouncer = new Debouncer(0.5, DebounceType.kFalling);

  private final Debouncer seeingTagForPoseResetDebouncer =
      new Debouncer(5.0, DebounceType.kFalling);

  private final TimeInterpolatableBuffer<Double> turretBuffer =
      TimeInterpolatableBuffer.createDoubleBuffer(2.0);

  private static final int STATIC_TURRET_CALIBRATION_FILTER_TAPS = 100;
  private final LinearFilter staticTurretCalibrationFilter = LinearFilter.movingAverage(20);
  private int currentStaticTurretCalibrationTap = 0;
  private double filteredTurretCalibration = 0;
  private boolean turretCalibrated = false;

  private final Imu imu;
  private final Limelight turretLimelight;
  private final Limelight backLimelight;
  private final Limelight groundLimelight;

  private OptionalTagResult turretResult = new OptionalTagResult();
  private OptionalTagResult adjustedTurretResult = new OptionalTagResult();
  private OptionalTagResult backResult = new OptionalTagResult();

  private double robotHeading;

  private double robotAngularVelocity;

  private boolean hasSeenTag = false;
  private boolean seeingTag = false;
  private boolean seeingTagDebounced = false;

  private boolean seenTagRecentlyForReset = true;
  private boolean seeingHubTags = false;

  public Vision(
      Imu imu, Limelight turretLimelight, Limelight backLimelight, Limelight groundLimelight) {
    super(SubsystemPriority.VISION, VisionState.TAGS);
    this.imu = imu;
    this.turretLimelight = turretLimelight;
    this.backLimelight = backLimelight;
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

    turretResult = turretLimelight.getTagResult();
    backResult = backLimelight.getTagResult();

    adjustedTurretResult = getAdjustedTurretLimelightTagResult(turretResult);

    if (turretResult.isPresent()) {
      hasSeenTag = true;
      seeingTag = true;
    } else {
      seeingTag = false;
    }
    seeingTagDebounced = seeingTagDebouncer.calculate(seeingTag);
    if (DriverStation.isDisabled()) {
      seenTagRecentlyForReset = true;
    } else {
      seenTagRecentlyForReset = seeingTagForPoseResetDebouncer.calculate(seeingTag);
    }

    seeingHubTags =
        seeingHubTagDebouncer.calculate(
            turretLimelight.seeingHubTag() || backLimelight.seeingHubTag());
  }

  // Call this in turret's periodic() or a fast telemetry thread
  public void addTurretObservation(double timestamp, double angle, double turretAngularVelocity) {
    turretBuffer.addSample(timestamp, MathHelpers.angleModulus(angle));
    turretLimelight.sendImuData(
        MathHelpers.angleModulus(robotHeading + angle),
        turretAngularVelocity + robotAngularVelocity,
        0.0,
        0.0,
        0.0,
        0.0);
  }

  private Optional<Double> getAngleAtTimestamp(double timestamp) {
    return turretBuffer.getSample(timestamp);
  }

  public void setEstimatedPoseAngle(double robotHeading) {
    this.robotHeading = robotHeading;
  }

  public OptionalTagResult getAdjustedTurretLimelighTagResult() {
    return adjustedTurretResult;
  }

  private OptionalTagResult getAdjustedTurretLimelightTagResult(OptionalTagResult turretResult) {
    if (turretResult.isEmpty()) {
      return adjustedTurretResult.empty();
    }

    var turretLimelightResult = turretResult.orElseThrow();

    var mT1Pose = turretLimelightResult.pose();
    var mT1Timestamp = turretLimelightResult.timestamp();
    var cameraToTurretTransform = VisionConfig.TURRET_TO_CAMERA.inverse();

    var fieldToTurretPose = mT1Pose.plus(cameraToTurretTransform);

    // Look up the turret angle at the specific image timestamp
    var robotToTurretObservation = getAngleAtTimestamp(mT1Timestamp);
    if (robotToTurretObservation.isEmpty()) {
      DogLog.logFault("Could not get turret angle at timestamp");
      return adjustedTurretResult.empty();
    }
    DogLog.log("Vision/TurretObservation", robotToTurretObservation.orElseThrow());
    DogLog.clearFault("Could not get turret angle at timestamp");

    // Create transform representing the rotation from Turret back to Robot
    // If the turret is at +90 degrees, we rotate -90 degrees to get back to the robot front.
    var turretToRobot =
        MathHelpers.transform2dFromRotation(
            Rotation2d.fromDegrees(robotToTurretObservation.orElseThrow()));

    // Add this rotation to the Turret's Field Pose to finally get the Robot's Field Pose
    var fieldToRobotEstimate = fieldToTurretPose.plus(turretToRobot);

    fieldToRobotEstimate = fieldToRobotEstimate.plus(TurretConfig.TURRET_TO_ROBOT.inverse());

    DogLog.log("Vision/AdjustedTurretPose", fieldToRobotEstimate);
    return adjustedTurretResult.update(
        fieldToRobotEstimate, mT1Timestamp, turretLimelightResult.standardDevs());
  }

  public OptionalTagResult getBackLimelightTagResult() {
    return backResult;
  }

  public boolean seeingTagDebounced() {
    return seeingTagDebounced || RobotBase.isSimulation();
  }

  public boolean seenTagRecentlyForReset() {
    return seenTagRecentlyForReset || RobotBase.isSimulation();
  }

  public boolean seeingTag() {
    return seeingTag || RobotBase.isSimulation();
  }

  public boolean hasSeenTag() {
    return hasSeenTag;
  }

  public void setState(VisionState state) {
    if (getState() == VisionState.CALIBRATE_STATIC_TURRET) {
      return;
    }
    if (state == VisionState.HUB_TAGS && getState() == VisionState.WAITING_FOR_HUB_TAGS) {
      return;
    }
    if (state == VisionState.HUB_TAGS) {
      state = VisionState.WAITING_FOR_HUB_TAGS;
    }

    setStateFromRequest(state);
  }

  public void calibrateTurretRequest() {
    if (!turretCalibrated) {
      setStateFromRequest(VisionState.CALIBRATE_STATIC_TURRET);
    }
  }

  public OptionalDouble getCalibratedTurretAngle() {
    if (turretCalibrated) {
      return OptionalDouble.of(filteredTurretCalibration);
    }
    return OptionalDouble.empty();
  }

  @Override
  protected void afterTransition(VisionState newState) {
    switch (newState) {
      case TAGS -> {
        turretLimelight.setState(LimelightState.TAGS);
        backLimelight.setState(LimelightState.TAGS);
        groundLimelight.setState(LimelightState.CLUSTER_MAP);
      }
      case HUB_TAGS -> {
        turretLimelight.setState(LimelightState.HUB_TAGS);
        backLimelight.setState(LimelightState.HUB_TAGS);
        groundLimelight.setState(LimelightState.CLUSTER_MAP);
      }
      case WAITING_FOR_HUB_TAGS -> {
        turretLimelight.setState(LimelightState.TAGS);
        backLimelight.setState(LimelightState.TAGS);
        groundLimelight.setState(LimelightState.CLUSTER_MAP);
      }
      case CALIBRATE_STATIC_TURRET -> {
        turretLimelight.setState(LimelightState.TAGS);
        backLimelight.setState(LimelightState.TAGS);
        groundLimelight.setState(LimelightState.CLUSTER_MAP);
      }
      default -> {}
    }
  }

  @Override
  public void whileInState(VisionState currentState) {
    // Send IMU data to all limelights
    // Set turret limelight angular velocity from turret
    backLimelight.sendImuData(robotHeading, robotAngularVelocity, 0.0, 0.0, 0.0, 0.0);

    DogLog.log("Vision/SeeingTag", seeingTag);
    DogLog.log("Vision/SeeingTagLast5Seconds", seenTagRecentlyForReset);

    switch (currentState) {
      case CALIBRATE_STATIC_TURRET -> {
        DogLog.logFault("CALIBRATING TURRET ANGLE", AlertType.kInfo);
        OptionalDouble maybeLimelightMegatagRotation = turretLimelight.getLimelightRotation();
        if (maybeLimelightMegatagRotation.isPresent()) {
          DogLog.log("TurretCal/CurrentTap", currentStaticTurretCalibrationTap);

          if (currentStaticTurretCalibrationTap == 0) {
            staticTurretCalibrationFilter.reset();
          }

          double limelightRotation = maybeLimelightMegatagRotation.getAsDouble();
          DogLog.log("TurretCal/FRTurretAngle", limelightRotation);
          var turretAngleRobotRelative = MathHelpers.angleModulus(limelightRotation - robotHeading);

          DogLog.log("TurretCal/RRTurretAngle", turretAngleRobotRelative);

          filteredTurretCalibration =
              staticTurretCalibrationFilter.calculate(turretAngleRobotRelative);

          DogLog.log("TurretCal/FilteredRRTurretAngle", filteredTurretCalibration);

          if (currentStaticTurretCalibrationTap == STATIC_TURRET_CALIBRATION_FILTER_TAPS) {
            turretCalibrated = true;
            DogLog.clearFault("CALIBRATING TURRET ANGLE");
            setStateFromRequest(VisionState.TAGS);
          }

          currentStaticTurretCalibrationTap++;
          DogLog.clearFault("TURRET CALIBRATION CAN'T SEE TAG");

        } else {
          DogLog.logFault("TURRET CALIBRATION CAN'T SEE TAG", AlertType.kInfo);
        }
      }
      default -> {}
    }

    if (turretCalibrated) {
      OptionalDouble maybeLimelightMegatagRotation = turretLimelight.getLimelightRotation();
      if (maybeLimelightMegatagRotation.isPresent()) {

        double limelightRotation = maybeLimelightMegatagRotation.getAsDouble();
        DogLog.log("TurretCal/FRTurretAngle", limelightRotation);
        var turretAngleRobotRelative = MathHelpers.angleModulus(limelightRotation - robotHeading);

        DogLog.log("TurretCal/RRTurretAngle", turretAngleRobotRelative);

        filteredTurretCalibration =
            staticTurretCalibrationFilter.calculate(turretAngleRobotRelative);

        DogLog.log("TurretCal/FilteredRRTurretAngle", filteredTurretCalibration);
      }
    }
  }
}
