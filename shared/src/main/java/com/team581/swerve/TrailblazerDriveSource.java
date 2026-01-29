package com.team581.swerve;

import com.team581.trailblazer.Trailblazer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.function.Supplier;

public class TrailblazerDriveSource implements DriveSource {
  private final Trailblazer trailblazer;
  private final Supplier<Pose2d> currentPose;
  private final Supplier<ChassisSpeeds> currentFieldRelativeSpeeds;

  public TrailblazerDriveSource(
      Trailblazer trailblazer,
      Supplier<Pose2d> currentPose,
      Supplier<ChassisSpeeds> currentFieldRelativeSpeeds) {
    this.trailblazer = trailblazer;
    this.currentPose = currentPose;
    this.currentFieldRelativeSpeeds = currentFieldRelativeSpeeds;
  }

  @Override
  public DriveSourceType getDriveSourceType() {
    return DriveSourceType.FIELD_CENTRIC_CLOSED_LOOP;
  }

  @Override
  public ChassisSpeeds getRequestedSpeeds() {
    return trailblazer.getFieldRelativeSetpoint(
        currentPose.get(), currentFieldRelativeSpeeds.get());
  }
}
