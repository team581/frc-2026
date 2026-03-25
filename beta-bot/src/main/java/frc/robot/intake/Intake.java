package frc.robot.intake;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;

public class Intake extends GenericIntake {
  private final TalonFX motor;
  private final NeutralOut neutralRequest = new NeutralOut();
  private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(false);

  public Intake(TalonFX motor) {
    motor.getConfigurator().apply(IntakeConfig.LEFT_MOTOR_CONFIG);
    this.motor = motor;
  }

  @Override
  public void shootRequest() {
    if (getState() == IntakeState.INTAKE) {
      return;
    }
    setStateFromRequest(IntakeState.SHOOT);
  }

  @Override
  public void shootThenIntakeRequest() {
    setStateFromRequest(IntakeState.SHOOT_THEN_INTAKE);
  }

  @Override
  public void stopShootingRequest() {
    switch (getState()) {
      case SHOOT -> setStateFromRequest(IntakeState.IDLE);
      case SHOOT_THEN_INTAKE -> setStateFromRequest(IntakeState.INTAKE);
      default -> {}
    }
  }

  @Override
  public void intakeRequest() {
    setStateFromRequest(IntakeState.INTAKE);
  }

  @Override
  public void intakeAutoRequest() {
    setStateFromRequest(IntakeState.INTAKE_AUTO);
  }

  @Override
  public void idleRequest() {
    setStateFromRequest(IntakeState.IDLE);
  }

  @Override
  protected void afterTransition(IntakeState newState) {
    switch (newState) {
      case IDLE -> motor.setControl(neutralRequest);
      case INTAKE -> motor.setControl(voltageRequest.withOutput(newState.getVoltage()));
      case INTAKE_AUTO -> motor.setControl(voltageRequest.withOutput(newState.getVoltage()));
      case SHOOT, SHOOT_THEN_INTAKE ->
          motor.setControl(voltageRequest.withOutput(newState.getVoltage()));
    }
  }

  @Override
  protected void collectInputs() {
    DogLog.log("Intake/StatorCurrent", motor.getStatorCurrent().getValueAsDouble());
    DogLog.log("Intake/VelocityRPM", motor.getVelocity().getValueAsDouble() * 60.0);
    DogLog.log("Intake/Voltage", getState().getVoltage());
  }

  @Override
  public void applyCurrentLimits(double supplyCurrentLimit) {
    motor
        .getConfigurator()
        .apply(
            IntakeConfig.LEFT_MOTOR_CONFIG.CurrentLimits.withSupplyCurrentLimit(
                supplyCurrentLimit));
  }
}
