package frc.robot.conveyor;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team581.mechanisms.PowerManaged;
import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import frc.robot.util.scheduling.SubsystemPriority;

public class Conveyor extends StateMachineSubsystem<ConveyorState> implements PowerManaged {

  private final TalonFX leftMotor;
  private final TalonFX rightMotor;
  private final NeutralOut neutralRequest = new NeutralOut();
  private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(false);

  public Conveyor(TalonFX leftMotor, TalonFX rightMotor) {
    super(SubsystemPriority.CONVEYOR, ConveyorState.IDLE);
    leftMotor.getConfigurator().apply(ConveyorConfig.LEFT_MOTOR_CONFIG);
    rightMotor.getConfigurator().apply(ConveyorConfig.RIGHT_MOTOR_CONFIG);

    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
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

  @Override
  protected void afterTransition(ConveyorState newState) {
    switch (newState) {
      case IDLE -> {
        leftMotor.setControl(neutralRequest);
        rightMotor.setControl(neutralRequest);
      }
      default -> {
        leftMotor.setControl(voltageRequest.withOutput(newState.getVoltage()));
        rightMotor.setControl(voltageRequest.withOutput(newState.getVoltage()));
      }
    }
  }

  @Override
  protected void collectInputs() {
    DogLog.log("Conveyor/Left/StatorCurrent", leftMotor.getStatorCurrent().getValueAsDouble());
    DogLog.log("Conveyor/Left/VelocityRPM", leftMotor.getVelocity().getValueAsDouble() * 60.0);
    DogLog.log("Conveyor/Right/StatorCurrent", rightMotor.getStatorCurrent().getValueAsDouble());
    DogLog.log("Conveyor/Left/SupplyCurrent", leftMotor.getSupplyCurrent().getValueAsDouble());
    DogLog.log("Conveyor/Right/SupplyCurrent", rightMotor.getSupplyCurrent().getValueAsDouble());
    DogLog.log("Conveyor/Right/VelocityRPM", rightMotor.getVelocity().getValueAsDouble() * 60.0);
    DogLog.log("Conveyor/Voltage", getState().getVoltage());
  }

  @Override
  public void applyCurrentLimits(double supplyCurrentLimit) {
    leftMotor
        .getConfigurator()
        .apply(
            ConveyorConfig.LEFT_MOTOR_CONFIG.CurrentLimits.withSupplyCurrentLimit(
                supplyCurrentLimit));
    rightMotor
        .getConfigurator()
        .apply(
            ConveyorConfig.RIGHT_MOTOR_CONFIG.CurrentLimits.withSupplyCurrentLimit(
                supplyCurrentLimit));
  }
}
