package com.team581.simkit;

import com.team581.simkit.internal.PositionMechanism;
import com.team581.simkit.internal.PositionMechanismBuilder;
import com.team581.simkit.internal.SimShooter;
import com.team581.simkit.internal.SimShooterBuilder;
import com.team581.simkit.internal.VelocityMechanism;
import com.team581.simkit.internal.VelocityMechanismBuilder;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.HashMap;
import java.util.Map;
import java.util.function.UnaryOperator;
import org.jspecify.annotations.Nullable;

/** Entry point for creating simple mechanism simulations. */
public final class SimKit {
  private static final Map<String, PositionMechanism> POSITION_MECHANISMS = new HashMap<>();
  private static final Map<String, VelocityMechanism> VELOCITY_MECHANISMS = new HashMap<>();
  private static final Map<String, SimShooter> SHOOTERS = new HashMap<>();

  public static @Nullable PositionMechanism positionMechanism(
      String name, UnaryOperator<PositionMechanismBuilder> factory) {
    if (RobotBase.isSimulation()) {
      return POSITION_MECHANISMS.computeIfAbsent(
          name, k -> factory.apply(new PositionMechanismBuilder()).build());
    }

    return null;
  }

  public static @Nullable SimShooter shooter(
      String name, UnaryOperator<SimShooterBuilder> factory) {
    if (RobotBase.isSimulation()) {
      return SHOOTERS.computeIfAbsent(
          name, k -> factory.apply(new SimShooterBuilder(name)).build());
    }

    return null;
  }

  public static @Nullable VelocityMechanism velocityMechanism(
      String name, UnaryOperator<VelocityMechanismBuilder> factory) {
    if (RobotBase.isSimulation()) {
      return VELOCITY_MECHANISMS.computeIfAbsent(
          name, k -> factory.apply(new VelocityMechanismBuilder()).build());
    }

    return null;
  }

  private SimKit() {}
}
