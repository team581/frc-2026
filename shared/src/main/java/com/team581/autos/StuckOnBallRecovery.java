package com.team581.autos;

import com.team581.trailblazer.AutoPoint;
import com.team581.trailblazer.Trailblazer;
import com.team581.trailblazer.segments.AutoSegment;
import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.networktables.DoubleSubscriber;
import java.util.function.Supplier;

public class StuckOnBallRecovery {
  private static final double STUCK_DEBOUNCE_SECONDS = 0.25;
  private static final Debouncer STUCK_DEBOUNCER = new Debouncer(STUCK_DEBOUNCE_SECONDS);
  private static final double STUCK_ANGLE_THRESHOLD = 7.0;

  private static final DoubleSubscriber RECOVERY_POINT_DISTANCE =
      DogLog.tunable("StuckOnBallRecovery/RecoveryPointDistance", 1.5);

  // For logging visualization only
  public static Pose2d getRecoveryPose(Pose2d robotPose, Rotation2d pitch, Rotation2d roll) {
    var headingToGetUnstuck =
        Rotation2d.fromRadians(Math.atan2(pitch.getRadians(), roll.getRadians()));
    return new Pose2d(
        robotPose
            .transformBy(new Transform2d(0.0, -RECOVERY_POINT_DISTANCE.get(), Rotation2d.kZero))
            .rotateAround(robotPose.getTranslation(), headingToGetUnstuck)
            .getTranslation(),
        robotPose.getRotation());
  }

  public static AutoSegment getRecoverySegment(
      Supplier<Pose2d> robotPose, Supplier<Rotation2d> pitch, Supplier<Rotation2d> roll) {
    return Trailblazer.segment(
            AutoPoint.of(
                () -> {
                  var headingToGetUnstuck =
                      Rotation2d.fromRadians(
                          Math.atan2(pitch.get().getRadians(), roll.get().getRadians()));
                  var recoveryPoint =
                      new Pose2d(
                          robotPose
                              .get()
                              .transformBy(
                                  new Transform2d(
                                      0.0, -RECOVERY_POINT_DISTANCE.get(), Rotation2d.kZero))
                              .rotateAround(robotPose.get().getTranslation(), headingToGetUnstuck)
                              .getTranslation(),
                          robotPose.get().getRotation());

                  return new Point(recoveryPoint, recoveryPoint);
                }))
        .forever();
  }

  public static boolean stuckOnBall(double pitch, double roll) {
    return STUCK_DEBOUNCER.calculate(Math.abs(Math.hypot(pitch, roll)) > STUCK_ANGLE_THRESHOLD);
  }
}
