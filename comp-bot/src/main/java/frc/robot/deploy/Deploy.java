package frc.robot.deploy;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.team581.mechanisms.PowerManaged;
import com.team581.simkit.SimKit;
import com.team581.util.state_machines.StateMachineSubsystem;
import com.team581.util.tuning.TunablePid;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.util.scheduling.SubsystemPriority;

public class Deploy extends StateMachineSubsystem<DeployState> implements PowerManaged {
  private final TalonFX motor;
  private final MotionMagicVoltage positionVoltageRequest =
      new MotionMagicVoltage(0).withEnableFOC(true);
  private final NeutralOut neutralRequest = new NeutralOut();
  private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);

  private double motorPosition = 0.0;
  private double statorCurrent = 0.0;
  private double supplyCurrent = 0.0;

  public Deploy(TalonFX motor) {
    super(SubsystemPriority.DEPLOY, DeployState.UNHOMED);
    this.motor = motor;

    TunablePid.register("Deploy", motor, DeployConfig.MOTOR_CONFIG);
  }

  public void intakeRequest() {
    switch (getState()) {
      case UNHOMED, HOME_INWARD, HOME_OUTWARD -> {
        // Do nothing, we aren't homed
      }
      default -> setStateFromRequest(DeployState.INTAKE);
    }
  }

  public void stowRequest() {
    switch (getState()) {
      case UNHOMED, HOME_INWARD, HOME_OUTWARD -> {
        // Do nothing, we aren't homed
      }
      default -> setStateFromRequest(DeployState.STOW);
    }
  }

  public boolean isFullyExtended() {
    return atGoal(DeployState.INTAKE);
  }

  public void homingRequest() {
    if (DriverStation.isAutonomous()) {
      setStateFromRequest(DeployState.HOME_INWARD);
    }
    setStateFromRequest(DeployState.HOME_OUTWARD);
  }

  public void homeInAutoRequest() {
    setStateFromRequest(DeployState.HOME_INWARD);
  }

  @Override
  protected DeployState getNextState(DeployState currentState) {
    return switch (currentState) {
      case UNHOMED, INTAKE, STOW -> currentState;

      case HOME_INWARD -> {
        if (motor.getStatorCurrent().getValueAsDouble() > DeployConfig.HOMING_CURRENT) {
          motor.setPosition(DeployConfig.HOMING_END_POSITION_INWARD);
          yield DeployState.INTAKE;
        } else {
          yield currentState;
        }
      }

      case HOME_OUTWARD -> {
        if (motor.getStatorCurrent().getValueAsDouble() > DeployConfig.HOMING_CURRENT) {
          motor.setPosition(DeployConfig.HOMING_END_POSITION_OUTWARD);
          yield DeployState.INTAKE;
        } else {
          yield currentState;
        }
      }
      default -> currentState;
    };
  }

  private static double clamp(double deployLength) {
    return MathUtil.clamp(deployLength, DeployConfig.MIN_LENGTH, DeployConfig.MAX_LENGTH);
  }

  @Override
  protected void afterTransition(DeployState newState) {
    switch (newState) {
      case UNHOMED -> motor.setControl(neutralRequest);
      case HOME_INWARD ->
          motor.setControl(voltageRequest.withOutput(DeployConfig.HOMING_VOLTAGE_INWARD));
      case HOME_OUTWARD ->
          motor.setControl(voltageRequest.withOutput(DeployConfig.HOMING_VOLTAGE_OUTWARD));
      default -> motor.setControl(positionVoltageRequest.withPosition(clamp(newState.getLength())));
    }
  }

  @Override
  protected void whileInState(DeployState state) {
    DogLog.log("Deploy/Position", motorPosition);
    DogLog.log("Deploy/GoalPosition", getState().getLength());
    DogLog.log("Deploy/StatorCurrent", statorCurrent);
    DogLog.log("Deploy/SupplyCurrent", supplyCurrent);
    DogLog.log("Deploy/Velocity", motor.getVelocity().getValueAsDouble());
    DogLog.log("Deploy/Voltage", motor.getMotorVoltage().getValueAsDouble());
  }

  public double getPosition() {
    return motorPosition;
  }

  public boolean atGoal() {
    return atGoal(getState());
  }

  public void hopperCompactionRequest() {
    switch (getState()) {
      case UNHOMED, HOME_INWARD, HOME_OUTWARD -> {}
      default -> setStateFromRequest(DeployState.HOPPER_COMPACTION_IN);
    }
  }

  public boolean atGoal(DeployState state) {
    return switch (state) {
      case UNHOMED, HOME_INWARD, HOME_OUTWARD -> false;
      default -> MathUtil.isNear(state.getLength(), motorPosition, DeployConfig.POSITION_TOLERANCE);
    };
  }

  @Override
  protected void collectInputs() {
    motorPosition = motor.getPosition().getValueAsDouble();
    statorCurrent = motor.getStatorCurrent().getValueAsDouble();
    supplyCurrent = motor.getSupplyCurrent().getValueAsDouble();
  }

  @Override
  public void simulationPeriodic() {
    var deploySimulation =
        SimKit.positionMechanism(
            "Deploy",
            mechanism ->
                mechanism
                    .addMotor(motor, ChassisReference.CounterClockwise_Positive)
                    .withMinPosition(DeployConfig.MIN_LENGTH)
                    .withMaxPosition(DeployConfig.MAX_LENGTH));

    if (getState() == DeployState.HOME_INWARD) {
      deploySimulation.seedPosition(DeployConfig.HOMING_END_POSITION_INWARD);
      setStateFromRequest(DeployState.INTAKE);
    }

    if (getState() == DeployState.HOME_OUTWARD) {
      deploySimulation.seedPosition(DeployConfig.HOMING_END_POSITION_OUTWARD);
      setStateFromRequest(DeployState.INTAKE);
    }

    deploySimulation.update();
  }

  @Override
  public void applyCurrentLimits(double supplyCurrentLimit) {
    motor
        .getConfigurator()
        .apply(DeployConfig.MOTOR_CONFIG.CurrentLimits.withSupplyCurrentLimit(supplyCurrentLimit));
  }
}
