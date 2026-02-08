package com.team581.trailblazer;

import com.team581.math.MathHelpers;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * Helper class for applying velocity constraints to path following outputs.
 *
 * <p>Handles both linear velocity constraints (using slew rate limiting) and angular velocity
 * constraints (using a profiled PID controller).
 */
public class ConstraintsCalculator {

  private final ProfiledPIDController profiledAngularController;

  private double lastTimestamp = MathSharedStore.getTimestamp();
  private double lastVelocity = 0.0;
  private double lastCommandedVelocity = 0.0;

  /**
   * Creates a new ConstraintsCalculator.
   *
   * @param angularPidConfig PID controller to copy gains from for angular velocity profiling
   */
  public ConstraintsCalculator(PIDController angularPidConfig) {
    this.profiledAngularController =
        new ProfiledPIDController(
            angularPidConfig.getP(),
            angularPidConfig.getI(),
            angularPidConfig.getD(),
            new AutoConstraintOptions().getAngularConstraints());

    this.profiledAngularController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Applies angular velocity constraints using a profiled PID controller.
   *
   * @param desiredAngularVelocity The desired angular velocity in rad/s (used when constraints are
   *     disabled)
   * @param currentAngleRadians The current heading in radians
   * @param targetAngleRadians The target heading in radians
   * @param currentSpeeds The current chassis speeds (used to reset profiled controller state)
   * @param constraints The constraint options to apply
   * @return The constrained angular velocity in rad/s
   */
  public double constrainAngularVelocity(
      double desiredAngularVelocity,
      double currentAngleRadians,
      double targetAngleRadians,
      ChassisSpeeds currentSpeeds,
      AutoConstraintOptions constraints) {
    if (constraints.maxAngularVelocity() > 0) {
      DogLog.timestamp("Constrainer/RunConstraint");
      return profiledAngularController.calculate(
          currentAngleRadians,
          new TrapezoidProfile.State(targetAngleRadians, 0),
          constraints.getAngularConstraints());
    }

    // Reset the profiled controller when constraints are not active
    profiledAngularController.reset(currentAngleRadians, currentSpeeds.omegaRadiansPerSecond);
    return desiredAngularVelocity;
  }

  /**
   * Applies linear velocity constraints using slew rate limiting.
   *
   * @param desiredVelocity The desired linear velocity in m/s
   * @param currentSpeeds The current chassis speeds (used to reset state when constraints are
   *     disabled)
   * @param constraints The constraint options to apply
   * @return The constrained linear velocity in m/s
   */
  public double constrainLinearVelocity(
      double desiredVelocity,
      ChassisSpeeds currentSpeeds,
      double distanceToEnd,
      AutoConstraintOptions constraints) {

    if (constraints.maxLinearVelocity() <= 0) {
      return desiredVelocity;
    }

    if (constraints.maxLinearAcceleration() <= 0) {
      return Math.min(constraints.maxLinearVelocity(), desiredVelocity);
    }

    var originalSign = Math.signum(desiredVelocity);
    desiredVelocity = Math.abs(desiredVelocity);
    var currentVelocity =
        Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);
    DogLog.log("Trailblazer/Follower/CurrentVelocity", currentVelocity);

    // Calculate how much we can accelerate this loop
    // New Velocity = Old Velocity + (Acceleration * dt)
    var currentTimestamp = MathSharedStore.getTimestamp();
    var kDt = Math.min(currentTimestamp - lastTimestamp, 0.04);
    var currentAccel = (currentVelocity - lastVelocity) / kDt;
    DogLog.log("Trailblazer/Follower/CurrentAccel", currentAccel);

    lastTimestamp = currentTimestamp;

    var maxReachableVelocity =
        Math.max((lastCommandedVelocity + (constraints.maxLinearAcceleration() * kDt)), 0.5);

    // Calculate velocity needed to stop in time
    var maxStoppingVelocity = Math.sqrt(2 * constraints.maxLinearAcceleration() * distanceToEnd);

    // Apply constraints
    var maxVelocityLimitedWantedVelocity =
        Math.min(desiredVelocity, constraints.maxLinearVelocity());

    var linearVelocity = Math.min(maxVelocityLimitedWantedVelocity, maxReachableVelocity);
    linearVelocity = Math.min(linearVelocity, maxStoppingVelocity);

    lastVelocity = currentVelocity;
    lastCommandedVelocity = linearVelocity;
    return Math.copySign(linearVelocity, originalSign);
  }

  /** Resets the internal state of the constrainer. */
  public void reset() {
    lastCommandedVelocity = 0.0;
    lastVelocity = 0.0;
    lastTimestamp = MathSharedStore.getTimestamp();
    profiledAngularController.reset(0, 0);
  }

  /**
   * Resets the internal state of the constrainer to match current robot state.
   *
   * @param currentSpeeds The current chassis speeds
   * @param currentAngleRadians The current heading in radians
   */
  public void reset(ChassisSpeeds currentSpeeds, double currentAngleRadians) {
    lastCommandedVelocity = MathHelpers.getLinearVelocity(currentSpeeds);
    lastVelocity = lastCommandedVelocity;
    lastTimestamp = MathSharedStore.getTimestamp();
    profiledAngularController.reset(currentAngleRadians, currentSpeeds.omegaRadiansPerSecond);
  }
}
