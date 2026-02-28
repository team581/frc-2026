package frc.robot.intake;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;

public enum IntakeState {
  INTAKE(11),
  IDLE(0),
  SHOOT(3);

  public final double velocity;
  public final DoubleSubscriber intakeTunableVoltage;

  IntakeState(double velocity) {
    this.velocity = velocity;
    this.intakeTunableVoltage = DogLog.tunable("Intake/" + this, velocity);
  }

  public double getIntakeVelocity() {
    return intakeTunableVoltage.get();
  }
}
