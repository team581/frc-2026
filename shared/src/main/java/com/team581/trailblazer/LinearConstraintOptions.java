package com.team581.trailblazer;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public record LinearConstraintOptions(
    /* Max linear velocity allowed in meters per second. Set to 0 to disable. */
    double maxVelocity,
    /* Max linear acceleration allowed in meters per second squared. Set to 0 to disable. */
    double maxAcceleration) {
  /** Default constraint options to use if no point or segment specific options are set. */
  public LinearConstraintOptions() {
    this(4.75, 4);
  }

  public TrapezoidProfile.Constraints getConstraints() {
    return new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration);
  }
}
