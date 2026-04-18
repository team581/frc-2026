package frc.robot.deploy;

import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.DifferentialMechanism;
import com.ctre.phoenix6.sim.ChassisReference;
import com.team581.math.MathHelpers;
import com.team581.mechanisms.PowerManaged;
import com.team581.simkit.SimKit;
import com.team581.util.state_machines.StateMachineSubsystem;
import com.team581.util.tuning.TunablePid;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.util.scheduling.SubsystemPriority;

public class Deploy extends StateMachineSubsystem<DeployState> implements PowerManaged {
  private final TalonFX leftMotor;
  private final TalonFX rightMotor;
  private final DifferentialMechanism<TalonFX> differentialMechanism;

  private final PositionTorqueCurrentFOC torqueCurrentFOCAverageRequest =
      new PositionTorqueCurrentFOC(0).withSlot(0);

  private final MotionMagicTorqueCurrentFOC motionMagicTorqueCurrentFOCAverageRequest =
      new MotionMagicTorqueCurrentFOC(0).withSlot(0);

  private final PositionTorqueCurrentFOC positionDifferentialRequest =
      new PositionTorqueCurrentFOC(0).withSlot(1);

  private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(false);
  private final VoltageOut voltageDifferentialRequest = new VoltageOut(0).withEnableFOC(false);

  private double differentialMechanismPosition = 0.0;
  private double leftMotorPosition = 0.0;
  private double rightMotorPosition = 0.0;
  private double leftStatorCurrent = 0.0;
  private double rightStatorCurrent = 0.0;
  private double leftSupplyCurrent = 0.0;
  private double rightSupplyCurrent = 0.0;

  public Deploy(DifferentialMechanism<TalonFX> differentialMechanism) {
    super(SubsystemPriority.DEPLOY, DeployState.UNHOMED);
    this.differentialMechanism = differentialMechanism;
    this.leftMotor = differentialMechanism.getLeader();
    this.rightMotor = differentialMechanism.getFollower();
    TunablePid.register("Deploy/Left", leftMotor, DeployConfig.LEFT_MOTOR_CONFIG);
    TunablePid.register("Deploy/Right", rightMotor, DeployConfig.RIGHT_MOTOR_CONFIG);

    differentialMechanism.setControl(
        torqueCurrentFOCAverageRequest.withPosition(0),
        positionDifferentialRequest.withPosition(0.0));
    differentialMechanism.setControl(
        motionMagicTorqueCurrentFOCAverageRequest.withPosition(0),
        positionDifferentialRequest.withPosition(0.0));
    differentialMechanism.setNeutralOut();
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
        if (leftStatorCurrent > DeployConfig.HOMING_CURRENT
            && rightStatorCurrent > DeployConfig.HOMING_CURRENT) {
          differentialMechanism.setPosition(DeployConfig.HOMING_END_POSITION_INWARD);
          yield DeployState.INTAKE;
        } else {
          yield currentState;
        }
      }

      case HOME_OUTWARD -> {
        if (leftStatorCurrent > DeployConfig.HOMING_CURRENT
            && rightStatorCurrent > DeployConfig.HOMING_CURRENT) {
          differentialMechanism.setPosition(DeployConfig.HOMING_END_POSITION_OUTWARD);
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
      case UNHOMED -> differentialMechanism.setNeutralOut();
      case HOME_INWARD ->
          differentialMechanism.setControl(
              voltageRequest.withOutput(DeployConfig.HOMING_VOLTAGE_INWARD),
              voltageDifferentialRequest.withOutput(0));
      case HOME_OUTWARD ->
          differentialMechanism.setControl(
              voltageRequest.withOutput(DeployConfig.HOMING_VOLTAGE_OUTWARD),
              voltageDifferentialRequest.withOutput(0));
      case SCORE_COMPACTION, SCORE_COMPACTION_WAITING -> {
        differentialMechanism.setControl(
            motionMagicTorqueCurrentFOCAverageRequest.withPosition(clamp(newState.getLength())),
            positionDifferentialRequest.withPosition(0.0));
      }
      default ->
          differentialMechanism.setControl(
              torqueCurrentFOCAverageRequest.withPosition(clamp(newState.getLength())),
              positionDifferentialRequest.withPosition(0.0));
    }
  }

  @Override
  protected void whileInState(DeployState state) {
    DogLog.log("Deploy/LeftMotor/Position", leftMotorPosition);
    DogLog.log("Deploy/RightMotor/Position", rightMotorPosition);
    DogLog.log("Deploy/GoalPosition", getState().getLength());
    DogLog.log("Deploy/DifferentialPosition", differentialMechanismPosition);
  }

  public double getPosition() {
    return differentialMechanismPosition;
  }

  public boolean atGoal() {
    return atGoal(getState());
  }

  public void waitHopperCompactionRequest() {
    switch (getState()) {
      case UNHOMED, HOME_INWARD, HOME_OUTWARD -> {}
      default -> setStateFromRequest(DeployState.SCORE_COMPACTION_WAITING);
    }
  }

  public void feedCompactionRequest() {
    switch (getState()) {
      case UNHOMED, HOME_INWARD, HOME_OUTWARD -> {}
      default -> setStateFromRequest(DeployState.FEED_COMPACTION);
    }
  }

  public void hopperCompactionRequest() {
    switch (getState()) {
      case UNHOMED, HOME_INWARD, HOME_OUTWARD -> {}
      default -> setStateFromRequest(DeployState.SCORE_COMPACTION);
    }
  }

  public void superCompactionRequest() {
    switch (getState()) {
      case UNHOMED, HOME_INWARD, HOME_OUTWARD -> {}
      default -> setStateFromRequest(DeployState.SUPER_COMPACTION_IN);
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
    leftMotorPosition = leftMotor.getPosition().getValueAsDouble();
    rightMotorPosition = rightMotor.getPosition().getValueAsDouble();
    if (RobotBase.isSimulation()) {
      // The firmware's DifferentialAveragePosition signal isn't reliably computed in sim,
      // so derive it from individual motor positions instead.
      differentialMechanismPosition = MathHelpers.average(leftMotorPosition, rightMotorPosition);
    } else {
      differentialMechanismPosition = differentialMechanism.getAveragePosition().getValueAsDouble();
    }
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
