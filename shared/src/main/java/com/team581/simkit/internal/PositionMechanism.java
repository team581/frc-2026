package com.team581.simkit.internal;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicDutyCycle;
import com.ctre.phoenix6.controls.DynamicMotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import java.util.List;
import java.util.Optional;
import java.util.OptionalDouble;

/** Predicts a simple Motion Magic style profile using TalonFX configuration state. */
public final class PositionMechanism {
  private static TrapezoidProfile.State currentMechanismState(List<SimMotor> devices) {
    var position =
        devices.stream()
            .mapToDouble(device -> device.motor().getPosition().getValueAsDouble())
            .average()
            .orElse(0.0);
    var velocity =
        devices.stream()
            .mapToDouble(device -> device.motor().getVelocity().getValueAsDouble())
            .average()
            .orElse(0.0);
    return new TrapezoidProfile.State(position, velocity);
  }

  private static TrapezoidProfile.State desiredMechanismState(List<SimMotor> devices) {
    var reference =
        devices.stream()
            .mapToDouble(device -> device.motor().getClosedLoopReference().getValueAsDouble())
            .average()
            .orElse(0.0);
    return new TrapezoidProfile.State(reference, 0.0);
  }

  private static TrapezoidProfile.Constraints getConfigConstraints(List<SimMotor> devices) {
    // Collect configs for averaging
    List<TalonFXConfiguration> configs =
        devices.stream()
            .map(
                motor -> {
                  var config = new TalonFXConfiguration();
                  motor.motor().getConfigurator().refresh(config);
                  return config;
                })
            .toList();

    var cruiseVelocity =
        configs.stream()
            .mapToDouble(c -> c.MotionMagic.MotionMagicCruiseVelocity)
            .average()
            .orElse(0.0);
    var acceleration =
        configs.stream()
            .mapToDouble(c -> c.MotionMagic.MotionMagicAcceleration)
            .average()
            .orElse(0.0);

    return new TrapezoidProfile.Constraints(cruiseVelocity, acceleration);
  }

  private static Optional<TrapezoidProfile.Constraints> getDynamicConstraints(
      List<SimMotor> devices) {
    // This assumes each motor is using the same control mode
    for (var device : devices) {
      var control = device.motor().getAppliedControl();

      if (control instanceof DynamicMotionMagicVoltage request) {
        return Optional.of(
            new TrapezoidProfile.Constraints(request.Velocity, request.Acceleration));
      }
      if (control instanceof DynamicMotionMagicDutyCycle request) {
        return Optional.of(
            new TrapezoidProfile.Constraints(request.Velocity, request.Acceleration));
      }
      if (control instanceof DynamicMotionMagicTorqueCurrentFOC request) {
        return Optional.of(
            new TrapezoidProfile.Constraints(request.Velocity, request.Acceleration));
      }
    }

    return Optional.empty();
  }

  private final List<SimMotor> devices;
  private final OptionalDouble minPosition;
  private final OptionalDouble maxPosition;
  private final Timer updateTimer = new Timer();
  private boolean hasRefreshedConfigConstraints = false;
  private TrapezoidProfile.Constraints configConstraints = new TrapezoidProfile.Constraints(0, 0);

  PositionMechanism(List<SimMotor> motors, OptionalDouble minPosition, OptionalDouble maxPosition) {
    this.devices = motors;
    this.minPosition = minPosition;
    this.maxPosition = maxPosition;
  }

  /** Seeds the rotor position of every motor to match the provided mechanism position. */
  public void seedPosition(double mechanismPosition) {
    for (var motor : devices) {
      motor.applyMechanismState(new TrapezoidProfile.State(mechanismPosition, 0.0));
    }
  }

  /** Recomputes the predicted state and pushes the result into each motor sim. */
  public void update() {
    update(desiredMechanismState(devices));
  }

  /**
   * Recomputes the predicted state using an explicit target position and pushes the result into
   * each motor sim. Use this for differential mechanisms where CTRE's sim doesn't correctly
   * populate ClosedLoopReference on individual motors.
   */
  public void update(double targetPosition) {
    update(new TrapezoidProfile.State(targetPosition, 0.0));
  }

  private TrapezoidProfile.State applyBounds(TrapezoidProfile.State state) {
    var clampedPosition =
        MathUtil.clamp(
            state.position,
            minPosition.orElse(Double.NEGATIVE_INFINITY),
            maxPosition.orElse(Double.POSITIVE_INFINITY));

    if (clampedPosition == state.position) {
      return state;
    }

    return new TrapezoidProfile.State(clampedPosition, 0.0);
  }

  private void update(TrapezoidProfile.State wantedState) {
    // Dynamic motion magic requests specify constraints per-request, so read them each update
    var constraints =
        getDynamicConstraints(devices)
            .orElseGet(
                () -> {
                  // Fall back to config-based constraints, cached after first read
                  if (!hasRefreshedConfigConstraints) {
                    hasRefreshedConfigConstraints = true;
                    configConstraints = getConfigConstraints(devices);
                  }
                  return configConstraints;
                });

    var currentState = currentMechanismState(devices);

    var predictedState =
        new TrapezoidProfile(constraints).calculate(updateTimer.get(), currentState, wantedState);

    // When disabled, overwrite predicted position to be current position and force 0 velocity
    if (DriverStation.isDisabled()) {
      predictedState = new TrapezoidProfile.State(currentState.position, 0.0);
    }

    var boundedState = applyBounds(predictedState);

    for (var motor : devices) {
      motor.applyMechanismState(boundedState);
    }

    updateTimer.restart();
  }
}
