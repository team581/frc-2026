package frc.robot.feeder;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team581.mechanisms.PowerManaged;
import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import frc.robot.util.scheduling.SubsystemPriority;

public class Feeder extends StateMachineSubsystem<FeederState> implements PowerManaged {

  private final TalonFX leftMotor;
  private final TalonFX rightMotor;
  private final NeutralOut neutralRequest = new NeutralOut();
  private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(false);

  public Feeder(TalonFX leftMotor, TalonFX rightMotor) {
    super(SubsystemPriority.FEEDER, FeederState.IDLE);
    leftMotor.getConfigurator().apply(FeederConfig.LEFT_MOTOR_CONFIG);
    rightMotor.getConfigurator().apply(FeederConfig.RIGHT_MOTOR_CONFIG);
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
  }

  public void shootRequest() {
    setStateFromRequest(FeederState.SHOOT);
  }

  public void idleRequest() {
    setStateFromRequest(FeederState.IDLE);
  }

  public void ejectRequest() {
    setStateFromRequest(FeederState.EJECT);
  }

  public void ballFillingRequest() {
    setStateFromRequest(FeederState.BALL_FILLING);
  }

  @Override
  protected void afterTransition(FeederState newState) {
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
    DogLog.log("Feeder/Left/StatorCurrent", leftMotor.getStatorCurrent().getValueAsDouble());
    DogLog.log("Feeder/Left/VelocityRPM", leftMotor.getVelocity().getValueAsDouble() * 60.0);
    DogLog.log("Feeder/Left/SupplyCurrent", leftMotor.getSupplyCurrent().getValueAsDouble());
    DogLog.log("Feeder/Right/StatorCurrent", rightMotor.getStatorCurrent().getValueAsDouble());
    DogLog.log("Feeder/Right/VelocityRPM", rightMotor.getVelocity().getValueAsDouble() * 60.0);
    DogLog.log("Feeder/Right/SupplyCurrent", rightMotor.getSupplyCurrent().getValueAsDouble());
    DogLog.log("Feeder/Voltage", getState().getVoltage());
  }

  @Override
  public void applyCurrentLimits(double supplyCurrentLimit) {
    leftMotor
        .getConfigurator()
        .apply(
            FeederConfig.LEFT_MOTOR_CONFIG.CurrentLimits.withSupplyCurrentLimit(
                supplyCurrentLimit));
    rightMotor
        .getConfigurator()
        .apply(
            FeederConfig.RIGHT_MOTOR_CONFIG.CurrentLimits.withSupplyCurrentLimit(
                supplyCurrentLimit));
  }
}
