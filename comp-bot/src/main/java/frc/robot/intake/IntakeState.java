package frc.robot.intake;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;

public enum IntakeState {
  INTAKE(12),
  IDLE(0),
  SHOOT(3);

  public final double voltage;
  public final DoubleSubscriber intakeTunableVoltage;

  IntakeState(double voltage) {
    this.voltage = voltage;
    this.intakeTunableVoltage = DogLog.tunable("Intake/" + this, voltage);
  }

  public double getIntakeVoltage() {
    return intakeTunableVoltage.get();
  }
}
