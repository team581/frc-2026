package com.team581.trailblazer;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public record AngularConstraintOptions(
    /* Max angular velocity allowed in radians per second. Set to 0 to disable. */
    double maxVelocity,
    /* Max angular acceleration allowed in radians per second squared. Set to 0 to disable. */
    double maxAcceleration) {
  /** Default constraint options to use if no point or segment specific options are set. */
  public AngularConstraintOptions() {
    this(Units.rotationsToRadians(4), Units.rotationsToRadians(4));
  }

  public TrapezoidProfile.Constraints getConstraints() {
    return new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration);
  }
}
