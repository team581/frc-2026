package com.team581.refaster;

import com.google.errorprone.refaster.annotation.AfterTemplate;
import com.google.errorprone.refaster.annotation.BeforeTemplate;
import edu.wpi.first.math.geometry.Rotation2d;

/** Refaster rules to prefer {@link Rotation2d} constants over factory methods for common angles. */
class Rotation2dRules {
  private Rotation2dRules() {}

  /** Prefer {@link Rotation2d#kZero} over equivalent factory methods and constructors. */
  static class Rotation2dZero {
    @BeforeTemplate
    Rotation2d fromDegrees() {
      return Rotation2d.fromDegrees(0.0);
    }

    @BeforeTemplate
    Rotation2d fromRadians() {
      return Rotation2d.fromRadians(0.0);
    }

    @BeforeTemplate
    Rotation2d constructorWithArg() {
      return new Rotation2d(0.0);
    }

    @BeforeTemplate
    Rotation2d constructorNoArg() {
      return new Rotation2d();
    }

    @AfterTemplate
    Rotation2d after() {
      return Rotation2d.kZero;
    }
  }

  /** Prefer {@link Rotation2d#kCCW_90deg} over equivalent factory methods and constructors. */
  static class Rotation2dCCW90 {
    @BeforeTemplate
    Rotation2d fromDegrees() {
      return Rotation2d.fromDegrees(90.0);
    }

    @BeforeTemplate
    Rotation2d fromRadians() {
      return Rotation2d.fromRadians(Math.PI / 2.0);
    }

    @BeforeTemplate
    Rotation2d constructor() {
      return new Rotation2d(Math.PI / 2.0);
    }

    @AfterTemplate
    Rotation2d after() {
      return Rotation2d.kCCW_90deg;
    }
  }

  /** Prefer {@link Rotation2d#kCW_90deg} over equivalent factory methods and constructors. */
  static class Rotation2dCW90 {
    @BeforeTemplate
    Rotation2d fromDegrees() {
      return Rotation2d.fromDegrees(-90.0);
    }

    @BeforeTemplate
    Rotation2d fromRadians() {
      return Rotation2d.fromRadians(-Math.PI / 2.0);
    }

    @BeforeTemplate
    Rotation2d constructor() {
      return new Rotation2d(-Math.PI / 2.0);
    }

    @AfterTemplate
    Rotation2d after() {
      return Rotation2d.kCW_90deg;
    }
  }

  /** Prefer {@link Rotation2d#k180deg} over equivalent factory methods and constructors. */
  static class Rotation2d180 {
    @BeforeTemplate
    Rotation2d fromDegreesPositive() {
      return Rotation2d.fromDegrees(180.0);
    }

    @BeforeTemplate
    Rotation2d fromDegreesNegative() {
      return Rotation2d.fromDegrees(-180.0);
    }

    @BeforeTemplate
    Rotation2d fromRadiansPositive() {
      return Rotation2d.fromRadians(Math.PI);
    }

    @BeforeTemplate
    Rotation2d fromRadiansNegative() {
      return Rotation2d.fromRadians(-Math.PI);
    }

    @BeforeTemplate
    Rotation2d constructorPositive() {
      return new Rotation2d(Math.PI);
    }

    @BeforeTemplate
    Rotation2d constructorNegative() {
      return new Rotation2d(-Math.PI);
    }

    @AfterTemplate
    Rotation2d after() {
      return Rotation2d.k180deg;
    }
  }
}
