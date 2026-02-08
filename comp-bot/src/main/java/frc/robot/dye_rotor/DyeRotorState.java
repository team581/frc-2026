package frc.robot.dye_rotor;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;

public enum DyeRotorState {
  IDLE(0.0, 0.0, 0.0),
  WARMUP(60.0, 0.0, 0.0),
  SHOOTING(150.0, 150.0, 0.0),
  UNJAM(0.0, 0.0, 0.0);

  public final double rotorRPM;
  public final double horizontalRPM;
  public final double verticalVoltage;

  private final DoubleSubscriber verticalVoltageTunable;

  private DyeRotorState(double rotorRPM, double horizontalRPM, double verticalVoltage) {
    this.rotorRPM = rotorRPM;
    this.horizontalRPM = horizontalRPM;
    this.verticalVoltage = verticalVoltage;

    this.verticalVoltageTunable = DogLog.tunable("DyeRotor/" + this, verticalVoltage);
  }

  public double getVerticalVoltage() {
    return verticalVoltageTunable.get();
  }
}
