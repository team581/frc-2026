package com.team581.math;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public record PoseErrorTolerance(double linearErrorTolerance, double angularErrorTolerance) {
  public PoseErrorTolerance(double linearErrorTolerance, Rotation2d angularErrorTolerance) {
    this(linearErrorTolerance, angularErrorTolerance.getDegrees());
  }

  public PoseErrorTolerance(double linearErrorTolerance) {
    this(linearErrorTolerance, 360);
  }

  public boolean atPose(Pose2d expected, Pose2d actual) {
    return atTranslation(expected.getTranslation(), actual.getTranslation())
        && atRotation(expected.getRotation(), actual.getRotation());
  }

  public boolean atTranslation(Translation2d expected, Translation2d actual) {
    var linearError = expected.getDistance(actual);

    return MathUtil.isNear(0, linearError, linearErrorTolerance);
  }

  public boolean atRotation(Rotation2d expected, Rotation2d actual) {
    return MathUtil.isNear(
        expected.getDegrees(), actual.getDegrees(), angularErrorTolerance, -180, 180);
  }
}
