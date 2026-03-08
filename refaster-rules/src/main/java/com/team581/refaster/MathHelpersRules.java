package com.team581.refaster;

import com.google.errorprone.refaster.annotation.AfterTemplate;
import com.google.errorprone.refaster.annotation.BeforeTemplate;
import com.team581.math.MathHelpers;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
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

  /**
   * Prefer {@link MathHelpers#angleModulus(double)} over {@code MathUtil.inputModulus(angle, -180,
   * 180)}.
   */
  static class AngleModulus {
    @BeforeTemplate
    double before(double angleDegrees) {
      return MathUtil.inputModulus(angleDegrees, -180, 180);
    }

    @AfterTemplate
    double after(double angleDegrees) {
      return MathHelpers.angleModulus(angleDegrees);
    }
  }

  /** Prefer {@link MathHelpers#average(double, double)} over {@code (a + b) / 2.0}. */
  static class Average {
    @BeforeTemplate
    double before(double a, double b) {
      return (a + b) / 2.0;
    }

    @AfterTemplate
    double after(double a, double b) {
      return MathHelpers.average(a, b);
    }
  }

  /**
   * Prefer {@link MathHelpers#rotation2d(double, double)} over {@code new Rotation2d(x, y)} to
   * safely handle small magnitude vectors.
   */
  static class Rotation2dFromXY {
    @BeforeTemplate
    Rotation2d before(double x, double y) {
      return new Rotation2d(x, y);
    }

    @AfterTemplate
    Rotation2d after(double x, double y) {
      return MathHelpers.rotation2d(x, y);
    }
  }
}
