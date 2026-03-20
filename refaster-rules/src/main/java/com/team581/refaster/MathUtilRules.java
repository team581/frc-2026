package com.team581.refaster;

import com.google.errorprone.refaster.annotation.AfterTemplate;
import com.google.errorprone.refaster.annotation.BeforeTemplate;
import edu.wpi.first.math.MathUtil;

/** Refaster rules to prefer {@link MathUtil} utility methods. */
class MathUtilRules {
  /**
   * Prefer {@link MathUtil#isNear(double, double, double)} over manually comparing {@code
   * Math.abs(a - b) <= tolerance}.
   */
  static class MathAbsIsNear {
    @AfterTemplate
    boolean after(double expected, double actual, double tolerance) {
      return MathUtil.isNear(expected, actual, tolerance);
    }

    @BeforeTemplate
    boolean before(double expected, double actual, double tolerance) {
      return Math.abs(expected - actual) < tolerance;
    }

    @BeforeTemplate
    boolean beforeEqual(double expected, double actual, double tolerance) {
      return Math.abs(expected - actual) <= tolerance;
    }
  }

  /**
   * Prefer {@link MathUtil#isNear(double, double, double)} over manually comparing {@code tolerance
   * >= Math.abs(a - b)}.
   */
  static class MathAbsIsNearReversed {
    @AfterTemplate
    boolean after(double expected, double actual, double tolerance) {
      return MathUtil.isNear(expected, actual, tolerance);
    }

    @BeforeTemplate
    boolean before(double expected, double actual, double tolerance) {
      return tolerance >= Math.abs(expected - actual);
    }

    @BeforeTemplate
    boolean beforeEqual(double expected, double actual, double tolerance) {
      return tolerance >= Math.abs(expected - actual);
    }
  }

  private MathUtilRules() {}
}
