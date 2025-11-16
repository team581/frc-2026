package com.team581.trailblazer;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface SwerveBase extends Subsystem {
  /**
   * Update the currently set Trailblazer speeds and request swerve to enter that state.
   *
   * @param speeds The speeds to request the robot to drive at (field relative).
   */
  void trailblazerDriveRequest(ChassisSpeeds speeds);

  ChassisSpeeds getFieldRelativeSpeeds();
}
