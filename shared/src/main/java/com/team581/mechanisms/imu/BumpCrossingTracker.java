package com.team581.mechanisms.imu;

import com.team581.autos.Point;
import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleSubscriber;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Provides a pose for path following that accounts for bump crossings. When the robot is tilted (on
 * the bump), returns a pose projected far ahead in the crossing direction so the path follower
 * commands full output. When the robot is on flat ground (debounced), returns the real target pose
 * so normal path following resumes.
 *
 * <p>The crossing direction is inferred from the robot's position relative to the target point.
 * Only points that use {@link #getPoint(Point)} are affected — regular {@code AutoPoint.ofRed()}
 * points bypass this entirely.
 */
public class BumpCrossingTracker {
  private static final double FLAT_DEBOUNCE_SECONDS = 0.1;
  private static final DoubleSubscriber FLAT_THRESHOLD =
      DogLog.tunable("BumpCrossing/FlatThresholdDegrees", 8.0);
  private static final DoubleSubscriber PROJECTION_DISTANCE_METERS =
      DogLog.tunable("BumpCrossing/ProjectionDistanceMeters", 5.0);

  private final Debouncer flatDebouncer =
      new Debouncer(FLAT_DEBOUNCE_SECONDS, DebounceType.kRising);
  private final DoubleSupplier pitchSupplier;
  private final DoubleSupplier rollSupplier;
  private final Supplier<Pose2d> robotPoseSupplier;
  private final Consumer<Translation2d> poseResetConsumer;
  private double bumpEnterDirectionalPitchSign = 0;
  private boolean enteredBump = false;
  private boolean exitedBump = false;

  /** Latched crossing direction: +1 or -1. 0 means not currently crossing. */
  private double latchedXSign = 0;

  public BumpCrossingTracker(
      DoubleSupplier pitchSupplier,
      DoubleSupplier rollSupplier,
      Supplier<Pose2d> robotPoseSupplier,
      Consumer<Translation2d> poseResetConsumer) {
    this.poseResetConsumer = poseResetConsumer;
    this.pitchSupplier = pitchSupplier;
    this.rollSupplier = rollSupplier;
    this.robotPoseSupplier = robotPoseSupplier;
  }

  /**
   * Get a {@link Point} adjusted for bump crossing. Use this as a pose supplier in {@code
   * AutoPoint.of(() -> tracker.getPoint(Point.ofRed(...)))}.
   *
   * @param point The base target point.
   * @param landingPoint The point on the field where the robot is expected to land after crossing.
   *     Used to help recover pose estimation.
   * @param driveDirection The drive direction to project our tilt onto
   * @return The point as-is if flat, or a projected point if on the bump.
   */
  public Point getPoint(Point point, Point landingPoint, Rotation2d driveDirection) {
    DogLog.log("Imu/BumpCrossing/OriginalPoint", point.getPose());
    // Get the tilt relative to the direction driving toward the bump
    double directionalTilt =
        (pitchSupplier.getAsDouble() * Math.cos(driveDirection.getRadians()))
            + (rollSupplier.getAsDouble() * Math.sin(driveDirection.getRadians()));
    boolean isFlat = flatDebouncer.calculate(Math.abs(directionalTilt) < FLAT_THRESHOLD.get());
    DogLog.log("Imu/BumpCrossing/IsFlatDebounced", isFlat);

    if (!enteredBump && Math.abs(directionalTilt) > FLAT_THRESHOLD.get()) {
      enteredBump = true;
      bumpEnterDirectionalPitchSign = Math.signum(directionalTilt);
    }

    if (enteredBump && !exitedBump) {
      if (bumpEnterDirectionalPitchSign > 0) {
        exitedBump = (directionalTilt < -FLAT_THRESHOLD.get());
      } else {
        exitedBump = (directionalTilt > FLAT_THRESHOLD.get());
      }
    }

    Pose2d targetPose = point.getPose();

    if (enteredBump && exitedBump && isFlat) {
      // We just crossed, reset pose
      poseResetConsumer.accept(landingPoint.getTranslation());
      enteredBump = false;
      exitedBump = false;
      DogLog.timestamp("Imu/BumpCrossing/CompletelyCrossedBump");
    }
    DogLog.log("Imu/BumpCrossing/EnteredBump", enteredBump);
    DogLog.log("Imu/BumpCrossing/ExitedBump", exitedBump);

    // previousIsFlat = isFlat;

    if (isFlat) {
      latchedXSign = 0;
      return point;
    }

    Pose2d robotPose = robotPoseSupplier.get();

    // Latch the crossing direction on the first tilted cycle so overshooting doesn't flip it.
    if (latchedXSign == 0) {
      latchedXSign = Math.signum(targetPose.getX() - robotPose.getX());
    }

    double xOffset = latchedXSign * PROJECTION_DISTANCE_METERS.get();

    Pose2d projected =
        new Pose2d(targetPose.getX() + xOffset, targetPose.getY(), targetPose.getRotation());

    DogLog.log("Imu/BumpCrossing/ProjectedPoint", projected);

    return new Point(projected, projected);
  }

  public void log() {
    DogLog.log("Imu/BumpCrossing/Pitch", pitchSupplier.getAsDouble());
    DogLog.log("Imu/BumpCrossing/Roll", rollSupplier.getAsDouble());
  }
}
