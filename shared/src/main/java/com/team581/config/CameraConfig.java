package com.team581.config;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public record CameraConfig(
    LimelightModel model,
    boolean useMt2,
    boolean useMt1RotationCloseUp,
    double forward,
    double right,
    double up,
    double pitch,
    double yaw,
    double roll) {

  public Transform3d getTransform3d() {
    return new Transform3d(
        new Translation3d(forward, right, up),
        new Rotation3d(Math.toRadians(roll), Math.toRadians(pitch), Math.toRadians(yaw)));
  }
}
