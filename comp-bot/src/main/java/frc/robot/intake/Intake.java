package frc.robot.intake;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.team581.simkit.SimKit;
import com.team581.util.state_machines.StateMachineSubsystem;
import com.team581.util.tuning.TunablePid;

import dev.doglog.DogLog;
import frc.robot.util.scheduling.SubsystemPriority;

public class Intake extends StateMachineSubsystem<IntakeState> {
  private final TalonFX motor;

  private final VelocityVoltage voltageRequest = new VelocityVoltage(0).withEnableFOC(false);

  public Intake(TalonFX motor) {
    super(SubsystemPriority.INTAKE, IntakeState.IDLE);

    motor.getConfigurator().apply(IntakeConfig.MOTOR_CONFIG);

        TunablePid.register("Shooter/LeftShooter", motor, IntakeConfig.MOTOR_CONFIG);

    this.motor = motor;
  }

  public void shootRequest() {
    if (getState() == IntakeState.INTAKE) {
      return;
    }
    setStateFromRequest(IntakeState.SHOOT);
  }

  public void intakeRequest() {
    setStateFromRequest(IntakeState.INTAKE);
  }

  public void idleRequest() {
    setStateFromRequest(IntakeState.IDLE);
  }

  @Override
  protected void afterTransition(IntakeState newState) {
    var wantedVelocity = newState.getIntakeVelocity() / 60.0;
    switch (newState) {
      case IDLE -> {
        motor.disable();
      }
      case INTAKE -> {
        motor.setControl(voltageRequest.withVelocity(wantedVelocity));
      }
      case SHOOT -> {
        motor.setControl(voltageRequest.withVelocity(wantedVelocity));
      }
    }
  }

  @Override
  protected void collectInputs() {
    DogLog.log("Intake/StatorCurrent", motor.getStatorCurrent().getValueAsDouble());
    DogLog.log("Intake/CurrentVelocity", motor.getVelocity().getValueAsDouble() * 60.0);
    DogLog.log("Intake/RequestedVelocity", getState().getIntakeVelocity());
  }


  @Override
  public void simulationPeriodic() {
    var intakeSimulation =
        SimKit.velocityMechanism(
            "intake",
            (mechanism) ->
                mechanism
                    .addMotor(motor, ChassisReference.CounterClockwise_Positive));

    intakeSimulation.update();
  }

}
