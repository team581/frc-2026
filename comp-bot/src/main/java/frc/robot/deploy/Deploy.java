package frc.robot.deploy;

import com.ctre.phoenix6.controls.DifferentialMotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.SimpleDifferentialMechanism;
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
  private final TalonFX leftMotor;
  private final TalonFX rightMotor;
  private final SimpleDifferentialMechanism<TalonFX> differentialMechanism;
  private final DifferentialMotionMagicVoltage differentialPositionVoltageRequest =
      new DifferentialMotionMagicVoltage(0, 0).withEnableFOC(false);

  private double differentialMechanismPosition = 0.0;
  private double leftMotorPosition = 0.0;
  private double rightMotorPosition = 0.0;
  private double leftStatorCurrent = 0.0;
  private double rightStatorCurrent = 0.0;
  private double leftSupplyCurrent = 0.0;
  private double rightSupplyCurrent = 0.0;

  public Deploy(SimpleDifferentialMechanism<TalonFX> differentialMechanism) {
    super(SubsystemPriority.DEPLOY, DeployState.UNHOMED);
    this.differentialMechanism = differentialMechanism;
    this.leftMotor = differentialMechanism.getLeader();
    this.rightMotor = differentialMechanism.getFollower();

    TunablePid.register("Deploy/Left", leftMotor, DeployConfig.LEFT_MOTOR_CONFIG);
    TunablePid.register("Deploy/Right", rightMotor, DeployConfig.RIGHT_MOTOR_CONFIG);
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

  public void stopShootingRequest() {
    setStateFromRequest(DeployState.STOW);
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
      // Do nothing
      case UNHOMED, INTAKE, STOW -> currentState;

      case HOME_INWARD -> {
        if (leftMotor.getStatorCurrent().getValueAsDouble() > DeployConfig.HOMING_CURRENT
            && rightMotor.getStatorCurrent().getValueAsDouble() > DeployConfig.HOMING_CURRENT) {
          differentialMechanism.setPosition(DeployConfig.HOMING_END_POSITION_INWARD);
          yield DeployState.INTAKE;
        } else {
          yield currentState;
        }
      }
      case HOME_OUTWARD -> {
        if (leftMotor.getStatorCurrent().getValueAsDouble() > DeployConfig.HOMING_CURRENT
            && rightMotor.getStatorCurrent().getValueAsDouble() > DeployConfig.HOMING_CURRENT) {
          differentialMechanism.setPosition(DeployConfig.HOMING_END_POSITION_OUTWARD);
          yield DeployState.INTAKE;
        } else {
          yield currentState;
        }
      }
      case HOPPER_COMPACTION_FINISH -> atGoal() ? DeployState.STOW : currentState;
      case HOPPER_COMPACTION_IN -> atGoal() ? DeployState.HOPPER_COMPACTION_FINISH : currentState;
    };
  }

  private static double clamp(double deployLength) {
    return MathUtil.clamp(deployLength, DeployConfig.MIN_LENGTH, DeployConfig.MAX_LENGTH);
  }

  @Override
  protected void afterTransition(DeployState newState) {
    switch (newState) {
      case UNHOMED -> {
        leftMotor.disable();
        rightMotor.disable();
      }
      case HOME_INWARD -> {
        leftMotor.setVoltage(DeployConfig.HOMING_VOLTAGE_INWARD);
        rightMotor.setVoltage(DeployConfig.HOMING_VOLTAGE_INWARD);
      }
      case HOME_OUTWARD -> {
        leftMotor.setVoltage(DeployConfig.HOMING_VOLTAGE_OUTWARD);
        rightMotor.setVoltage(DeployConfig.HOMING_VOLTAGE_OUTWARD);
      }
      default ->
          differentialMechanism.setControl(
              differentialPositionVoltageRequest
                  .withAveragePosition(clamp(newState.getLength()))
                  .withDifferentialPosition(0));
    }
  }

  @Override
  protected void whileInState(DeployState state) {
    DogLog.log("Deploy/LeftMotor/Position", leftMotorPosition);
    DogLog.log("Deploy/RightMotor/Position", rightMotorPosition);
    DogLog.log("Deploy/GoalPosition", getState().getLength());
    DogLog.log("Deploy/DifferentialPosition", differentialMechanismPosition);
    DogLog.log("Deploy/LeftMotor/StatorCurrent", leftStatorCurrent);
    DogLog.log("Deploy/LeftMotor/SupplyCurrent", leftSupplyCurrent);
    DogLog.log("Deploy/RightMotor/StatorCurrent", rightStatorCurrent);
    DogLog.log("Deploy/RightMotor/SupplyCurrent", rightSupplyCurrent);
    DogLog.log("Deploy/LeftMotor/Velocity", leftMotor.getVelocity().getValueAsDouble());
    DogLog.log("Deploy/RightMotor/Velocity", rightMotor.getVelocity().getValueAsDouble());
    DogLog.log("Deploy/RightMotor/Voltage", rightMotor.getMotorVoltage().getValueAsDouble());
    DogLog.log("Deploy/LeftMotor/Voltage", leftMotor.getMotorVoltage().getValueAsDouble());
    DogLog.log("Deploy/LeftMotor/Slot", leftMotor.getClosedLoopSlot().getValueAsDouble());
    DogLog.log("Deploy/RightMotor/Slot", rightMotor.getClosedLoopSlot().getValueAsDouble());
  }

  public double getPosition() {
    return differentialMechanismPosition;
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
      default ->
          MathUtil.isNear(state.getLength(), leftMotorPosition, DeployConfig.POSITION_TOLERANCE)
              && MathUtil.isNear(
                  state.getLength(), rightMotorPosition, DeployConfig.POSITION_TOLERANCE);
    };
  }

  @Override
  protected void collectInputs() {
    differentialMechanismPosition = differentialMechanism.getAveragePosition().getValueAsDouble();
    leftMotorPosition = leftMotor.getPosition().getValueAsDouble();
    rightMotorPosition = rightMotor.getPosition().getValueAsDouble();
    leftStatorCurrent = leftMotor.getStatorCurrent().getValueAsDouble();
    rightStatorCurrent = rightMotor.getStatorCurrent().getValueAsDouble();
    leftSupplyCurrent = leftMotor.getSupplyCurrent().getValueAsDouble();
    rightSupplyCurrent = rightMotor.getSupplyCurrent().getValueAsDouble();
  }

  @Override
  public void simulationPeriodic() {
    var deploySimulation =
        SimKit.positionMechanism(
            "Deploy",
            mechanism ->
                mechanism
                    .addMotor(leftMotor, ChassisReference.Clockwise_Positive)
                    .addMotor(rightMotor, ChassisReference.CounterClockwise_Positive)
                    .withMinPosition(DeployConfig.MIN_LENGTH)
                    .withMaxPosition(DeployConfig.MAX_LENGTH));

    if (getState() == DeployState.HOME_INWARD) {
      // Use seedPosition instead of differentialMechanism.setPosition to avoid creating a
      // firmware-level sensor offset that compounds with setRawRotorPosition in
      // applyMechanismState.
      deploySimulation.seedPosition(DeployConfig.HOMING_END_POSITION_INWARD);
      setStateFromRequest(DeployState.INTAKE);
    }
    if (getState() == DeployState.HOME_OUTWARD) {
      deploySimulation.seedPosition(DeployConfig.HOMING_END_POSITION_OUTWARD);
      setStateFromRequest(DeployState.INTAKE);
    }

    deploySimulation.update(clamp(getState().getLength()));
  }

  @Override
  public void applyCurrentLimits(double supplyCurrentLimit) {
    leftMotor
        .getConfigurator()
        .apply(
            DeployConfig.LEFT_MOTOR_CONFIG.CurrentLimits.withSupplyCurrentLimit(
                supplyCurrentLimit));
    rightMotor
        .getConfigurator()
        .apply(
            DeployConfig.RIGHT_MOTOR_CONFIG.CurrentLimits.withSupplyCurrentLimit(
                supplyCurrentLimit));
  }
}
