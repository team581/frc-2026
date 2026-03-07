package frc.robot.intake;

import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;

public class IntakeTwoMotor extends GenericIntake {
  private final TalonFX leftMotor;
  private final TalonFX rightMotor;

  public IntakeTwoMotor(TalonFX leftMotor, TalonFX rightMotor) {
    leftMotor.getConfigurator().apply(IntakeConfig.LEFT_MOTOR_CONFIG);
    rightMotor.getConfigurator().apply(IntakeConfig.RIGHT_MOTOR_CONFIG);
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
  }

  public void shootRequest() {
    if (getState() == IntakeState.INTAKE) {
      return;
    }
    setStateFromRequest(IntakeState.SHOOT);
  }

  public void shootThenIntakeRequest() {
    setStateFromRequest(IntakeState.SHOOT_THEN_INTAKE);
  }

  public void stopShootingRequest() {
    switch (getState()) {
      case SHOOT -> setStateFromRequest(IntakeState.IDLE);
      case SHOOT_THEN_INTAKE -> setStateFromRequest(IntakeState.INTAKE);
      default -> {}
    }
  }

  public void intakeRequest() {
    setStateFromRequest(IntakeState.INTAKE);
  }

  public void intakeAutoRequest() {
    setStateFromRequest(IntakeState.INTAKE_AUTO);
  }

  public void idleRequest() {
    setStateFromRequest(IntakeState.IDLE);
  }

  @Override
  protected void afterTransition(IntakeState newState) {
    switch (newState) {
      case IDLE -> {
        leftMotor.disable();
        rightMotor.disable();
      }
      case INTAKE -> {
        leftMotor.setVoltage(newState.getIntakeVoltage());
        rightMotor.setVoltage(newState.getIntakeVoltage());
      }
      case INTAKE_AUTO -> {
        leftMotor.setVoltage(newState.getIntakeVoltage());
        rightMotor.setVoltage(newState.getIntakeVoltage());
      }
      case SHOOT, SHOOT_THEN_INTAKE -> {
        leftMotor.setVoltage(newState.getIntakeVoltage());
        rightMotor.setVoltage(newState.getIntakeVoltage());
      }
    }
  }

  @Override
  protected void collectInputs() {
    DogLog.log("Intake/Left/StatorCurrent", leftMotor.getStatorCurrent().getValueAsDouble());
    DogLog.log("Intake/Left/VelocityRPM", leftMotor.getVelocity().getValueAsDouble() * 60.0);
    DogLog.log("Intake/Right/StatorCurrent", rightMotor.getStatorCurrent().getValueAsDouble());
    DogLog.log("Intake/Right/VelocityRPM", rightMotor.getVelocity().getValueAsDouble() * 60.0);
    DogLog.log("Intake/Voltage", getState().getIntakeVoltage());
  }
}
