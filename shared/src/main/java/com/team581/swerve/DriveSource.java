package com.team581.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * A drive source is an abstraction over something that outputs requested chassis speeds. These
 * speeds may be followed exactly (ex. driving normally in teleop) or might be processed (ex.
 * snapping to face a specific angle).
 *
 * <p>Realistically, the only implementations of this are {@link XboxControllerDriveSource} and
 * {@link TrailblazerDriveSource}.
 */
public interface DriveSource {
  DriveSourceType getDriveSourceType();

  ChassisSpeeds getRequestedSpeeds();
}
