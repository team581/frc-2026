package com.team581.refaster;

import com.google.errorprone.refaster.annotation.AfterTemplate;
import com.google.errorprone.refaster.annotation.BeforeTemplate;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Refaster rules to prefer {@link Pose2d} constants over factory methods for common values. */
class Pose2dRules {
  /** Prefer {@link Pose2d#kZero} over equivalent constructors. */
  static class Pose2dZero {
    @AfterTemplate
    Pose2d after() {
      return Pose2d.kZero;
    }

    @BeforeTemplate
    Pose2d constructorNoArg() {
      return new Pose2d();
    }

    @BeforeTemplate
    Pose2d constructorWithCoordinatesAndRotationConstant(Rotation2d rotation) {
      return new Pose2d(0.0, 0.0, rotation);
    }

    @BeforeTemplate
    Pose2d constructorWithTranslationAndRotation(Translation2d translation) {
      return new Pose2d(Translation2d.kZero, Rotation2d.kZero);
    }
  }

  private Pose2dRules() {}
}
