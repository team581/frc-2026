package com.team581.mechanisms.imu;

import com.team581.autos.Point;
import com.team581.util.state_machines.StateMachine;
import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleSubscriber;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

public class BumpCrossingTracker extends StateMachine<BumpCrossingState> {
  private static final double FLAT_DEBOUNCE_SECONDS = 0.04;
  private static final double FLAT_FALLBACK_DEBOUNCE_SECONDS = 0.75;
  private static final double STUCK_IN_STATE_TIMEOUT_DURATION = 2.5;
  private static final DoubleSubscriber FLAT_THRESHOLD =
      DogLog.tunable("BumpCrossing/FlatThresholdDegrees", 5.0);
  private static final DoubleSubscriber UPHILL_THRESHOLD =
      DogLog.tunable("BumpCrossing/UphillThresholdDegrees", 5.0);
  private static final DoubleSubscriber DOWNHILL_THRESHOLD =
      DogLog.tunable("BumpCrossing/DownhillThresholdDegrees", -2.0);

  private final Debouncer flatDebouncer =
      new Debouncer(FLAT_DEBOUNCE_SECONDS, DebounceType.kRising);
  private final Debouncer flatFallbackDebouncer =
      new Debouncer(FLAT_FALLBACK_DEBOUNCE_SECONDS, DebounceType.kRising);
  private final DoubleSupplier pitchSupplier;
  private final DoubleSupplier rollSupplier;
  private final DoubleSupplier headingSupplier;
  private final Consumer<Translation2d> poseResetConsumer;
  private Rotation2d crossingDirection = Rotation2d.kZero;
  private double directionalTilt = 0.0;
  private boolean isFlat = true;
  private boolean isFlatFallbackDebounced = false;
  private Point landingPoint = new Point(Pose2d.kZero, Pose2d.kZero);
  private Rotation3d robotRelativeImuState = Rotation3d.kZero;

  public BumpCrossingTracker(
      DoubleSupplier pitchSupplier,
      DoubleSupplier rollSupplier,
      DoubleSupplier headingSupplier,
      Consumer<Translation2d> poseResetConsumer) {
    super(BumpCrossingState.FLAT_NOT_CROSSING);
    this.poseResetConsumer = poseResetConsumer;
    this.pitchSupplier = pitchSupplier;
    this.rollSupplier = rollSupplier;
    this.headingSupplier = headingSupplier;
  }

  /**
   * Calculates the signed tilt component along the crossing direction.
   *
   * <p>Pitch and roll are robot-frame, while crossing direction is field-frame. We rotate the
   * body-up unit vector ({@code (0, 0, 1)}) by the robot's orientation to get body-up in the field
   * frame, then rotate it again by the negative crossing direction so the crossing direction
   * becomes the +X axis. The asin of the X component of the resulting vector is the tilt along the
   * crossing direction: positive when tilted up toward the crossing direction, negative when tilted
   * away.
   */
  public static double calculateDirectionalTilt(
      double pitchDegrees,
      double rollDegrees,
      double headingDegrees,
      Rotation2d crossingDirection) {
    var robotOrientation =
        new Rotation3d(
            Math.toRadians(rollDegrees),
            Math.toRadians(pitchDegrees),
            Math.toRadians(headingDegrees));
    var bodyUpInCrossingFrame =
        new Translation3d(0.0, 0.0, 1.0)
            .rotateBy(robotOrientation)
            .rotateBy(new Rotation3d(0.0, 0.0, -crossingDirection.getRadians()));
    return Math.toDegrees(Math.asin(bodyUpInCrossingFrame.getX()));
  }

  @Override
  protected void collectInputs() {
    var pitchDegrees = pitchSupplier.getAsDouble();
    var rollDegrees = rollSupplier.getAsDouble();
    var headingDegrees = headingSupplier.getAsDouble();
    robotRelativeImuState =
        new Rotation3d(
            Math.toRadians(rollDegrees),
            Math.toRadians(pitchDegrees),
            Math.toRadians(headingDegrees));
    // Get the tilt relative to the known crossing direction (set via bumpCrossRequest)
    // Positive tilt should be tilted up toward the crossing direction
    directionalTilt =
        calculateDirectionalTilt(pitchDegrees, rollDegrees, headingDegrees, crossingDirection);
    isFlat = flatDebouncer.calculate(Math.abs(directionalTilt) < FLAT_THRESHOLD.get());
    isFlatFallbackDebounced =
        flatFallbackDebouncer.calculate(Math.abs(directionalTilt) < FLAT_THRESHOLD.get());
  }

  private BumpCrossingState stuckInStateTimeout() {
    poseResetConsumer.accept(landingPoint.getTranslation());
    DogLog.timestamp("Imu/BumpCrossing/FinishedCrossing/StuckInStateTimeout");
    return BumpCrossingState.FLAT_NOT_CROSSING;
  }

  @Override
  public BumpCrossingState getNextState(BumpCrossingState currentState) {
    // Fallback
    if (currentState == BumpCrossingState.CROSSING_UPHILL && isFlatFallbackDebounced) {
      poseResetConsumer.accept(landingPoint.getTranslation());
      DogLog.timestamp("Imu/BumpCrossing/FinishedCrossing/FallbackDebounced");
      return BumpCrossingState.FLAT_NOT_CROSSING;
    }

    return switch (currentState) {
      case FLAT_ABOUT_TO_CROSS -> {
        if (directionalTilt > UPHILL_THRESHOLD.get()) {
          yield BumpCrossingState.CROSSING_UPHILL;
        }
        if (timeout(STUCK_IN_STATE_TIMEOUT_DURATION)) {
          yield stuckInStateTimeout();
        }
        yield currentState;
      }
      case CROSSING_UPHILL -> {
        if (directionalTilt < DOWNHILL_THRESHOLD.get()) {
          yield BumpCrossingState.CROSSING_DOWNHILL;
        }
        if (timeout(STUCK_IN_STATE_TIMEOUT_DURATION)) {
          yield stuckInStateTimeout();
        }
        yield currentState;
      }
      case CROSSING_DOWNHILL -> {
        if (isFlat) {
          yield BumpCrossingState.FLAT_NOT_CROSSING;
        }
        if (timeout(STUCK_IN_STATE_TIMEOUT_DURATION)) {
          yield stuckInStateTimeout();
        }
        yield currentState;
      }
      case FLAT_NOT_CROSSING -> currentState;
    };
  }

  public void bumpCrossRequest(Point landingPoint, Rotation2d crossingDirection) {
    this.landingPoint = landingPoint;
    this.crossingDirection = crossingDirection;
    DogLog.timestamp("Imu/BumpCrossing/CrossRequest");
    setStateFromRequest(BumpCrossingState.FLAT_ABOUT_TO_CROSS);
  }

  @Override
  protected void beforeTransition(BumpCrossingState oldState, BumpCrossingState newState) {
    if (oldState == BumpCrossingState.CROSSING_DOWNHILL
        && newState == BumpCrossingState.FLAT_NOT_CROSSING) {
      // We just crossed, reset pose
      poseResetConsumer.accept(landingPoint.getTranslation());
      DogLog.timestamp("Imu/BumpCrossing/FinishedCrossing/NormalSequence");
    }
  }

  public void log() {
    DogLog.log("Imu/BumpCrossing/State", getState());
    DogLog.log("Imu/BumpCrossing/CrossingDirection", crossingDirection);
    DogLog.log(
        "Imu/BumpCrossing/RobotRelativeImuState", new Pose3d(0.0, 0.0, 0.0, robotRelativeImuState));
    DogLog.log("Imu/BumpCrossing/DirectionalTilt", directionalTilt);
    DogLog.log("Imu/BumpCrossing/IsFlat", isFlat);
    DogLog.log("Imu/BumpCrossing/IsFlatFallbackDebounced", isFlatFallbackDebounced);
  }
}
