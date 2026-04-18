package com.team581.mechanisms.imu;

import com.team581.autos.Point;
import com.team581.math.MathHelpers;
import com.team581.util.state_machines.StateMachine;
import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoubleSubscriber;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

public class BumpCrossingTracker extends StateMachine<BumpCrossingState> {
  private static final double FLAT_DEBOUNCE_SECONDS = 0.1;
  private static final double FLAT_FALLBACK_DEBOUNCE_SECONDS = 1.0;
  private static final DoubleSubscriber FLAT_THRESHOLD =
      DogLog.tunable("BumpCrossing/FlatThresholdDegrees", 3.0);
  private static final DoubleSubscriber CROSSING_THRESHOLD =
      DogLog.tunable("BumpCrossing/CrossingThresholdDegrees", 7.0);

  private final Debouncer flatDebouncer =
      new Debouncer(FLAT_DEBOUNCE_SECONDS, DebounceType.kRising);
  private final Debouncer flatFallbackDebouncer =
      new Debouncer(FLAT_FALLBACK_DEBOUNCE_SECONDS, DebounceType.kRising);
  private final DoubleSupplier pitchSupplier;
  private final DoubleSupplier rollSupplier;
  private final Consumer<Translation2d> poseResetConsumer;
  private ChassisSpeeds currentSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
  private Rotation2d driveDirection = Rotation2d.kZero;
  private double directionalTilt = 0.0;
  private boolean isFlat = true;
  private boolean isFlatFallbackDebounced = false;
  private Point landingPoint;

  public BumpCrossingTracker(
      DoubleSupplier pitchSupplier,
      DoubleSupplier rollSupplier,
      Consumer<Translation2d> poseResetConsumer) {
    super(BumpCrossingState.NOT_ON_BUMP);
    this.poseResetConsumer = poseResetConsumer;
    this.pitchSupplier = pitchSupplier;
    this.rollSupplier = rollSupplier;
  }

  @Override
  protected void collectInputs() {
    // Get the tilt relative to the direction driving toward the bump
    driveDirection = MathHelpers.getDriveDirection(currentSpeeds);
    directionalTilt =
        -((pitchSupplier.getAsDouble() * Math.cos(driveDirection.getRadians()))
            + (rollSupplier.getAsDouble() * Math.sin(driveDirection.getRadians())));
    isFlat = flatDebouncer.calculate(Math.abs(directionalTilt) < FLAT_THRESHOLD.get());
    isFlatFallbackDebounced =
        flatFallbackDebouncer.calculate(Math.abs(directionalTilt) < FLAT_THRESHOLD.get());
  }

  @Override
  public BumpCrossingState getNextState(BumpCrossingState currentState) {
    // Fallback
    if (currentState == BumpCrossingState.CROSSING_UPHILL && isFlatFallbackDebounced) {
      poseResetConsumer.accept(landingPoint.getTranslation());
      DogLog.timestamp("Imu/BumpCrossing/FallbackFinishedCrossing");
      return BumpCrossingState.NOT_ON_BUMP;
    }

    return switch (currentState) {
      case NOT_ON_BUMP -> {
        if (directionalTilt > CROSSING_THRESHOLD.get()) {
          yield BumpCrossingState.CROSSING_UPHILL;
        }
        yield currentState;
      }
      case CROSSING_UPHILL -> {
        if (directionalTilt < -CROSSING_THRESHOLD.get()) {
          yield BumpCrossingState.CROSSING_DOWNHILL;
        }
        yield currentState;
      }
      case CROSSING_DOWNHILL -> {
        if (isFlat) {
          yield BumpCrossingState.NOT_ON_BUMP;
        }
        yield currentState;
      }
    };
  }

  public void bumpCrossRequest(Point landingPoint) {
    this.landingPoint = landingPoint;
    DogLog.timestamp("Imu/BumpCrossing/CrossRequest");
  }

  public void setCurrentSpeeds(ChassisSpeeds speeds) {
    currentSpeeds = speeds;
  }

  @Override
  protected void beforeTransition(BumpCrossingState oldState, BumpCrossingState newState) {
    if (oldState == BumpCrossingState.CROSSING_DOWNHILL
        && newState == BumpCrossingState.NOT_ON_BUMP) {
      // We just crossed, reset pose
      poseResetConsumer.accept(landingPoint.getTranslation());
      DogLog.timestamp("Imu/BumpCrossing/FinishedCrossing");
    }
  }

  public void log() {
    DogLog.log("Imu/BumpCrossing/State", getState());
    DogLog.log("Imu/BumpCrossing/DirectionalTilt", directionalTilt);
    DogLog.log("Imu/BumpCrossing/IsFlat", isFlat);
  }
}
