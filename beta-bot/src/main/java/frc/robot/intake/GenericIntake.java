package frc.robot.intake;

import com.team581.mechanisms.PowerManaged;
import com.team581.util.state_machines.StateMachineSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;

public abstract class GenericIntake extends StateMachineSubsystem<IntakeState>
    implements PowerManaged {
  protected GenericIntake() {
    super(SubsystemPriority.INTAKE, IntakeState.IDLE);
  }

  public abstract void idleRequest();

  public abstract void intakeAutoRequest();

  public abstract void intakeRequest();

  public abstract void shootRequest();

  public abstract void shootThenIntakeRequest();

  public abstract void stopShootingRequest();
}
