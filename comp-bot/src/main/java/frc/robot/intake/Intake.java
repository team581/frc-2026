package frc.robot.intake;

import com.ctre.phoenix6.hardware.TalonFX;
import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import frc.robot.util.scheduling.SubsystemPriority;

public class Intake extends StateMachineSubsystem<IntakeState> {
  private final TalonFX motor;

  public Intake(TalonFX motor) {
    super(SubsystemPriority.INTAKE, IntakeState.IDLE);

    motor.getConfigurator().apply(IntakeConfig.MOTOR_CONFIG);
    this.motor = motor;
  }

  public void shootRequest() {
    setStateFromRequest(IntakeState.SHOOT);
  }

  public void intakeRequest() {
    setStateFromRequest(IntakeState.INTAKE);
  }

  public void idleRequest() {
    setStateFromRequest(IntakeState.IDLE);
  }

  @Override
  protected void whileInState(IntakeState state) {
    // TODO: Remove after bringup
    afterTransition(state);
  }

  @Override
  protected void afterTransition(IntakeState newState) {
    switch (newState) {
      case IDLE -> {
        motor.disable();
      }
      case INTAKE -> {
        motor.setVoltage(newState.getIntakeVoltage());
      }
      case SHOOT -> {
        motor.setVoltage(newState.getIntakeVoltage());
      }
    }
  }

  @Override
  protected void collectInputs() {
    DogLog.log("Intake/StatorCurrent", motor.getStatorCurrent().getValueAsDouble());
    DogLog.log("Intake/VelocityRPM", motor.getVelocity().getValueAsDouble() * 60.0);
    DogLog.log("Intake/Voltage", getState().voltage);
  }
}
