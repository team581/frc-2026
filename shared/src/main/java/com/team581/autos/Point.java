package com.team581.autos;

import com.team581.math.MathHelpers;
import com.team581.math.PoseErrorTolerance;
import com.team581.trailblazer.AutoPoint;
import com.team581.util.FmsUtil;
import edu.wpi.first.math.geometry.Pose2d;

public record Point(Pose2d redPose, Pose2d bluePose) {
  public static Point ofRed(Pose2d redPose) {
    return new Point(redPose, MathHelpers.pathflip(redPose));
  }

  public static Point ofBlue(Pose2d bluePose) {
    return new Point(MathHelpers.pathflip(bluePose), bluePose);
  }

  public Pose2d getPose() {
    return FmsUtil.isRedAlliance() ? redPose : bluePose;
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
