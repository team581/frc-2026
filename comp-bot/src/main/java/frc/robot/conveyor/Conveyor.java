package frc.robot.conveyor;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team581.mechanisms.PowerManaged;
import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import frc.robot.util.scheduling.SubsystemPriority;

public class Conveyor extends StateMachineSubsystem<ConveyorState> implements PowerManaged {

  private final TalonFX topMotor;
  private final TalonFX bottomMotor;
  private final NeutralOut neutralRequest = new NeutralOut();
  private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);

  public Conveyor(TalonFX topMotor, TalonFX bottomMotor) {
    super(SubsystemPriority.CONVEYOR, ConveyorState.IDLE);
    topMotor.getConfigurator().apply(ConveyorConfig.TOP_MOTOR_CONFIG);
    bottomMotor.getConfigurator().apply(ConveyorConfig.BOTTOM_MOTOR_CONFIG);

    this.topMotor = topMotor;
    this.bottomMotor = bottomMotor;
  }

  public void shootRequest() {
    setStateFromRequest(ConveyorState.SHOOT);
  }

  public void idleRequest() {
    setStateFromRequest(ConveyorState.IDLE);
  }

  public void ballFillingRequest() {
    setStateFromRequest(ConveyorState.BALL_FILLING);
  }

  public void ejectRequest() {
    setStateFromRequest(ConveyorState.EJECT);
  }

  public void intakeRequest() {
    setStateFromRequest(ConveyorState.INTAKE);
  }

  @Override
  protected void afterTransition(ConveyorState newState) {
    switch (newState) {
      case IDLE -> {
        topMotor.setControl(neutralRequest);
        bottomMotor.setControl(neutralRequest);
      }
      default -> {
        topMotor.setControl(voltageRequest.withOutput(newState.getVoltage()));
        bottomMotor.setControl(voltageRequest.withOutput(newState.getVoltage()));
      }
    }
  }

  @Override
  protected void collectInputs() {
    DogLog.log("Conveyor/Top/StatorCurrent", topMotor.getStatorCurrent().getValueAsDouble());
    DogLog.log("Conveyor/Top/VelocityRPM", topMotor.getVelocity().getValueAsDouble() * 60.0);
    DogLog.log("Conveyor/Bottom/StatorCurrent", bottomMotor.getStatorCurrent().getValueAsDouble());
    DogLog.log("Conveyor/Top/SupplyCurrent", topMotor.getSupplyCurrent().getValueAsDouble());
    DogLog.log("Conveyor/Bottom/SupplyCurrent", bottomMotor.getSupplyCurrent().getValueAsDouble());
    DogLog.log("Conveyor/Bottom/VelocityRPM", bottomMotor.getVelocity().getValueAsDouble() * 60.0);
    DogLog.log("Conveyor/Voltage", getState().getVoltage());
    DogLog.log("Conveyor/State", getState().name());
  }

  @Override
  public void applyCurrentLimits(double supplyCurrentLimit) {
    topMotor
        .getConfigurator()
        .apply(
            ConveyorConfig.TOP_MOTOR_CONFIG.CurrentLimits.withSupplyCurrentLimit(
                supplyCurrentLimit));
    bottomMotor
        .getConfigurator()
        .apply(
            ConveyorConfig.BOTTOM_MOTOR_CONFIG.CurrentLimits.withSupplyCurrentLimit(
                supplyCurrentLimit));
  }
}
