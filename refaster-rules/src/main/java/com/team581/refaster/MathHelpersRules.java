package com.team581.refaster;

import com.google.errorprone.refaster.annotation.AfterTemplate;
import com.google.errorprone.refaster.annotation.BeforeTemplate;
import com.team581.math.MathHelpers;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** Refaster rules to prefer {@link MathHelpers} utility methods. */
class MathHelpersRules {
  private MathHelpersRules() {}

  /**
   * Prefer {@link MathHelpers#getLinearVelocity(ChassisSpeeds)} over manually computing {@code
   * Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond)}.
   */
  static class ChassisSpeedsLinearVelocity {
    @BeforeTemplate
    double before(ChassisSpeeds speeds) {
      return Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    }

    @AfterTemplate
    double after(ChassisSpeeds speeds) {
      return MathHelpers.getLinearVelocity(speeds);
    }
  }
}
