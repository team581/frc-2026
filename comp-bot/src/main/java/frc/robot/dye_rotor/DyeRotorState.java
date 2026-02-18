package frc.robot.dye_rotor;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;

public enum DyeRotorState {
  IDLE(0.0, 0.0, 0.0),
  SHOOT(20.0, 10.0, 10.0),
  UNJAM(0.0, 0.0, 0.0);

  private static double bpsToRpm(double bps) {
    var circumference = 18.0 * Math.PI;
    var ballDiameter = 6.0;
    return (bps / (circumference / ballDiameter)) * 60;
  }

  public final double rotorRPM;
  public final double horizontalVoltage;
  public final double verticalVoltage;

  private final DoubleSubscriber rotorBPSTunable;
  private final DoubleSubscriber verticalVoltageTunable;
  private final DoubleSubscriber horizontalVoltageTunable;

  private DyeRotorState(double rotorBPS, double horizontalVoltage, double verticalVoltage) {
    this.rotorRPM = bpsToRpm(rotorBPS);
    this.horizontalVoltage = horizontalVoltage;
    this.verticalVoltage = verticalVoltage;

    this.rotorBPSTunable = DogLog.tunable("DyeRotor/RotorBallsPerSecond/" + this, rotorBPS);
    this.verticalVoltageTunable =
        DogLog.tunable("DyeRotor/VerticalVoltage/" + this, verticalVoltage);
    this.horizontalVoltageTunable =
        DogLog.tunable("DyeRotor/HorizontalVoltage/" + this, horizontalVoltage);
  }

  public double getHorizontalVoltage() {
    return horizontalVoltageTunable.get();
  }

  public double getRotorRPM() {
    return bpsToRpm(rotorBPSTunable.get());
  }

  public double getVerticalVoltage() {
    return verticalVoltageTunable.get();
  }
}
