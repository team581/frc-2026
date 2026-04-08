package frc.robot.feeder;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team581.mechanisms.PowerManaged;
import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import frc.robot.util.scheduling.SubsystemPriority;

public class Feeder extends StateMachineSubsystem<FeederState> implements PowerManaged {

  private final TalonFX topMotor;
  private final TalonFX bottomMotor;
  private final NeutralOut neutralRequest = new NeutralOut();
  private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);

  public Feeder(TalonFX topMotor, TalonFX bottomMotor) {
    super(SubsystemPriority.FEEDER, FeederState.IDLE);
    topMotor.getConfigurator().apply(FeederConfig.TOP_MOTOR_CONFIG);
    bottomMotor.getConfigurator().apply(FeederConfig.BOTTOM_MOTOR_CONFIG);
    this.topMotor = topMotor;
    this.bottomMotor = bottomMotor;
  }

  public void shootRequest() {
    setStateFromRequest(FeederState.SHOOT);
  }

  public void intakeRequest() {
    setStateFromRequest(FeederState.INTAKING);
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
    DogLog.log("Feeder/Top/StatorCurrent", topMotor.getStatorCurrent().getValueAsDouble());
    DogLog.log("Feeder/Top/VelocityRPM", topMotor.getVelocity().getValueAsDouble() * 60.0);
    DogLog.log("Feeder/Top/SupplyCurrent", topMotor.getSupplyCurrent().getValueAsDouble());
    DogLog.log("Feeder/Bottom/StatorCurrent", bottomMotor.getStatorCurrent().getValueAsDouble());
    DogLog.log("Feeder/Bottom/VelocityRPM", bottomMotor.getVelocity().getValueAsDouble() * 60.0);
    DogLog.log("Feeder/Bottom/SupplyCurrent", bottomMotor.getSupplyCurrent().getValueAsDouble());
    DogLog.log("Feeder/Voltage", getState().getVoltage());
  }

  @Override
  public void applyCurrentLimits(double supplyCurrentLimit) {
    topMotor
        .getConfigurator()
        .apply(
            FeederConfig.TOP_MOTOR_CONFIG.CurrentLimits.withSupplyCurrentLimit(supplyCurrentLimit));
    bottomMotor
        .getConfigurator()
        .apply(
            FeederConfig.BOTTOM_MOTOR_CONFIG.CurrentLimits.withSupplyCurrentLimit(
                supplyCurrentLimit));
  }
}
