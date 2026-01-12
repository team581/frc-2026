package com.team581.math;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class PoseLookahead {

  public static Pose2d getLookaheadPose(Pose2d current, ChassisSpeeds velocity, double lookahead) {
    var x = current.getX() + velocity.vxMetersPerSecond * lookahead;
    var y = current.getY() + velocity.vyMetersPerSecond * lookahead;
    var theta =
        current
            .getRotation()
            .plus(Rotation2d.fromRadians(velocity.omegaRadiansPerSecond * lookahead));

    return new Pose2d(x, y, theta);
  }

  public ChassisSpeeds getTangentialVelocity(Pose2d current, ChassisSpeeds robotVelocity) {
    // Tangential velocity = robot velocity - radial velocity, relative to the goal
    return new ChassisSpeeds();
  }

  public PoseLookahead() {}
}
