package frc.robot.intake;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team581.mechanisms.PowerManaged;
import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import frc.robot.util.scheduling.SubsystemPriority;

public class Intake extends StateMachineSubsystem<IntakeState> implements PowerManaged {
  private final TalonFX leftMotor;
  private final TalonFX rightMotor;
  private final NeutralOut neutralRequest = new NeutralOut();
  private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(false);

  public Intake(TalonFX leftMotor, TalonFX rightMotor) {
    super(SubsystemPriority.INTAKE, IntakeState.IDLE);

    leftMotor.getConfigurator().apply(IntakeConfig.LEFT_MOTOR_CONFIG);
    rightMotor.getConfigurator().apply(IntakeConfig.RIGHT_MOTOR_CONFIG);
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
  }

  public void ejectRequest() {
    setStateFromRequest(IntakeState.EJECT);
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

  public boolean hasBeenIntaking() {
    if (getState() == IntakeState.INTAKE && timeout(3)) {
      return true;
    }
    return false;
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

  public void idleRequest() {
    setStateFromRequest(IntakeState.IDLE);
  }

  @Override
  protected void afterTransition(IntakeState newState) {
    switch (newState) {
      case IDLE -> {
        leftMotor.setControl(neutralRequest);
        rightMotor.setControl(neutralRequest);
      }
      default -> {
        leftMotor.setControl(voltageRequest.withOutput(newState.voltage));
        rightMotor.setControl(voltageRequest.withOutput(newState.voltage));
      }
    }
  }

  @Override
  protected void collectInputs() {
    DogLog.log("Intake/Left/StatorCurrent", leftMotor.getStatorCurrent().getValueAsDouble());
    DogLog.log("Intake/Right/StatorCurrent", rightMotor.getStatorCurrent().getValueAsDouble());
    DogLog.log("Intake/Left/SupplyCurrent", leftMotor.getSupplyCurrent().getValueAsDouble());
    DogLog.log("Intake/Right/SupplyCurrent", rightMotor.getSupplyCurrent().getValueAsDouble());
    DogLog.log("Intake/Left/VelocityRPM", leftMotor.getVelocity().getValueAsDouble() * 60.0);
    DogLog.log("Intake/Right/VelocityRPM", rightMotor.getVelocity().getValueAsDouble() * 60.0);
    DogLog.log("Intake/RequestedVoltage", getState().getVoltage());
    DogLog.log("Intake/HasBeenIntaking", hasBeenIntaking());
  }

  @Override
  public void applyCurrentLimits(double supplyCurrentLimit) {
    leftMotor
        .getConfigurator()
        .apply(
            IntakeConfig.LEFT_MOTOR_CONFIG.CurrentLimits.withSupplyCurrentLimit(
                supplyCurrentLimit));
    rightMotor
        .getConfigurator()
        .apply(
            IntakeConfig.RIGHT_MOTOR_CONFIG.CurrentLimits.withSupplyCurrentLimit(
                supplyCurrentLimit));
  }
}
