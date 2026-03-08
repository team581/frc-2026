package com.team581.autos;

import com.team581.config.FeatureFlag;
import com.team581.math.PoseErrorTolerance;
import com.team581.trailblazer.AutoPoint;
import com.team581.util.FieldUtil;
import com.team581.util.FmsUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import java.util.function.BooleanSupplier;

public record Point(Pose2d redPose, Pose2d bluePose) {
  // Also determines whether or not we are using our HOME FIELD for swerve assist
  public static final BooleanSupplier CLAMPED_POINTS_FEATURE_FLAG =
      FeatureFlag.of("ClampedAutoPoints", true);

  private static final Rectangle2d CLAMPED_AREA =
      new Rectangle2d(
          new Translation2d(16.54, 8.069),
          new Translation2d(8.246 + Units.inchesToMeters(31), 1.645));

  private static Pose2d clamp(Pose2d input) {
    return new Pose2d(CLAMPED_AREA.nearest(input.getTranslation()), input.getRotation());
  }

  public static Point ofRed(Pose2d redPose) {
    return new Point(redPose, FieldUtil.pathflip(redPose));
  }

  public static Point ofBlue(Pose2d bluePose) {
    return new Point(FieldUtil.pathflip(bluePose), bluePose);
  }

  public Pose2d getPose() {
    var result = FmsUtil.isRedAlliance() ? redPose : bluePose;

    if (CLAMPED_POINTS_FEATURE_FLAG.getAsBoolean()) {
      return clamp(result);
    }

    return result;
  }

  public Translation2d getTranslation() {
    return getPose().getTranslation();
  }

  public double getX() {
    return getPose().getX();
  }

  public double getY() {
    return getPose().getY();
  }

  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  public Point plus(Transform2d other) {
    return new Point(redPose.plus(other), bluePose.plus(other));
  }

  public Point transformBy(Transform2d other) {
    return new Point(redPose.transformBy(other), bluePose.transformBy(other));
  }

  public Point rotateBy(Rotation2d other) {
    return new Point(redPose.rotateBy(other), bluePose.rotateBy(other));
  }

  public AutoPoint withLinearConstraints(double maxVelocity, double maxAcceleration) {
    return AutoPoint.of(this).withLinearConstraints(maxVelocity, maxAcceleration);
  }

  public AutoPoint withAngularConstraints(
      double maxAngularVelocity, double maxAngularAcceleration) {
    return AutoPoint.of(this).withAngularConstraints(maxAngularVelocity, maxAngularAcceleration);
  }

  public AutoPoint withTransitionTolerance(PoseErrorTolerance transitionTolerance) {
    return AutoPoint.of(this).withTransitionTolerance(transitionTolerance);
  }
}
