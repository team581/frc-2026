package com.team581.simkit.internal;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.team581.math.SlewRateLimiterStateless;
import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.List;
import java.util.OptionalDouble;

/** Predicts a simple velocity-controlled mechanism using TalonFX configuration state. */
public final class VelocityMechanism {
  private static double desiredMechanismVelocity(List<SimMotor> devices) {
    return devices.stream()
        .mapToDouble(device -> device.motor().getClosedLoopReference().getValueAsDouble())
        .average()
        .orElse(0.0);
  }

  private static double getMechanismAccelerationLimit(List<SimMotor> devices) {
    // Collect configs for averaging
    var configs =
        devices.stream()
            .map(
                motor -> {
                  var config = new TalonFXConfiguration();
                  motor.motor().getConfigurator().refresh(config);
                  return config;
                })
            .toList();

    return configs.stream()
        .mapToDouble(c -> c.MotionMagic.MotionMagicAcceleration)
        .average()
        .orElse(0.0);
  }

  private final List<SimMotor> devices;
  private final OptionalDouble minVelocity;
  private final OptionalDouble maxVelocity;
  private double currentVelocity = 0.0;
  private double previousTimestamp = MathSharedStore.getTimestamp();
  private boolean hasRefreshedAccelerationLimit = false;
  private double accelerationLimit = 0.0;

  VelocityMechanism(List<SimMotor> motors, OptionalDouble minVelocity, OptionalDouble maxVelocity) {
    this.devices = motors;
    this.minVelocity = minVelocity;
    this.maxVelocity = maxVelocity;
  }

  /** Recomputes the predicted velocity and pushes the result into each motor sim. */
  public void update() {
    if (!hasRefreshedAccelerationLimit) {
      hasRefreshedAccelerationLimit = true;
      accelerationLimit = getMechanismAccelerationLimit(devices);
    }

    currentVelocity =
        devices.stream()
            .mapToDouble(device -> device.motor().getVelocity().getValueAsDouble())
            .average()
            .orElseThrow();

    var wantedVelocity = desiredMechanismVelocity(devices);
    // When disabled, target 0 velocity to simulate spin-down from lack of voltage
    var boundedWantedVelocity =
        DriverStation.isDisabled() ? 0.0 : applyVelocityBounds(wantedVelocity);
    var newVelocity =
        SlewRateLimiterStateless.calculate(
            boundedWantedVelocity,
            currentVelocity,
            previousTimestamp,
            accelerationLimit,
            -accelerationLimit);

    var now = MathSharedStore.getTimestamp();
    var dt = now - previousTimestamp;
    previousTimestamp = now;

    for (var motor : devices) {
      motor.applyVelocity(newVelocity, dt);
    }
  }

  private double applyVelocityBounds(double velocity) {
    return MathUtil.clamp(
        velocity,
        minVelocity.orElse(Double.NEGATIVE_INFINITY),
        maxVelocity.orElse(Double.POSITIVE_INFINITY));
  }
}
