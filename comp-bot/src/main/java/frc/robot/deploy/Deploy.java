package frc.robot.deploy;

import com.ctre.phoenix6.controls.DifferentialMotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.SimpleDifferentialMechanism;
import com.ctre.phoenix6.sim.ChassisReference;
import com.team581.simkit.SimKit;
import com.team581.util.state_machines.StateMachineSubsystem;
import com.team581.util.tuning.TunablePid;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.config.DSOptions;
import frc.robot.config.FeatureFlags;
import frc.robot.util.scheduling.SubsystemPriority;

public class Deploy extends StateMachineSubsystem<DeployState> {
  private final TalonFX leftMotor;
  private final TalonFX rightMotor;
  private final SimpleDifferentialMechanism<TalonFX> differentialMechanism;
  private final CANrange hopperCANRange;
  private final LinearFilter hopperFilter = LinearFilter.movingAverage(5);
  private final DifferentialMotionMagicVoltage differentialPositionVoltageRequest =
      new DifferentialMotionMagicVoltage(0, 0).withEnableFOC(false);

  private HopperCapacity hopperCapacity = HopperCapacity.LOW;
  private double differentialMechanismPosition = 0.0;
  private double leftMotorPosition = 0.0;
  private double rightMotorPosition = 0.0;
  private double leftStatorCurrent = 0.0;
  private double rightStatorCurrent = 0.0;
  private double leftSupplyCurrent = 0.0;
  private double rightSupplyCurrent = 0.0;
  private double hopperCANRangeDistance = 0.0;
  private double previousCanRangeDistance = 0.0;
  private double filteredHopperCANRangeDistance;
  private boolean hopperCapacityNotHigh = false;

  private final Timer hopperShuffleTimer = new Timer();
  private final Timer canRangeUpdateTimer = new Timer();

  public Deploy(
      SimpleDifferentialMechanism<TalonFX> differentialMechanism, CANrange hopperCANRange) {
    super(SubsystemPriority.DEPLOY, DeployState.UNHOMED);
    this.differentialMechanism = differentialMechanism;
    this.leftMotor = differentialMechanism.getLeader();
    this.rightMotor = differentialMechanism.getFollower();
    this.hopperCANRange = hopperCANRange;

    hopperShuffleTimer.start();
    canRangeUpdateTimer.start();

    hopperCANRange.getConfigurator().apply(DeployConfig.CAN_RANGE_CONFIG);

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

  public void shuffleRequest() {
    switch (getState()) {
      case UNHOMED, HOME_INWARD, HOME_OUTWARD -> {
        // Do nothing, we aren't homed
      }
      default -> {
        if (FeatureFlags.HOPPER_SHUFFLING.getAsBoolean()
            && getState() != DeployState.HOPPER_SHUFFLING_FINISH
            && getState() != DeployState.HOPPER_SHUFFLING_OUT
            && getState() != DeployState.HOPPER_SHUFFLING_IN) {
          hopperShuffleTimer.reset();
          setStateFromRequest(DeployState.HOPPER_SHUFFLING_OUT);
          hopperShuffleTimer.restart();
        }
      }
    }
  }

  public void homingRequest() {
    if (DriverStation.isAutonomous()) {
      setStateFromRequest(DeployState.HOME_INWARD);
    }
    setStateFromRequest(DeployState.HOME_OUTWARD);
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
      case HOPPER_SHUFFLING_OUT -> {
        if (hopperShuffleTimer.hasElapsed(DeployConfig.HOPPER_SHUFFLE_DURATION.get())) {
          yield DeployState.HOPPER_SHUFFLING_FINISH;
        }
        if ((atGoal() || timeout(2.0)) && hopperCapacityNotHigh) {
          yield DeployState.HOPPER_SHUFFLING_IN;
        }
        yield currentState;
      }

      case HOPPER_SHUFFLING_IN -> {
        if (hopperShuffleTimer.hasElapsed(DeployConfig.HOPPER_SHUFFLE_DURATION.get())) {
          yield DeployState.HOPPER_SHUFFLING_FINISH;
        }
        if ((atGoal() || timeout(2.0)) && hopperCapacityNotHigh) {
          yield DeployState.HOPPER_SHUFFLING_OUT;
        }
        yield currentState;
      }
      case HOPPER_SHUFFLING_FINISH -> {
        if (atGoal()) {
          hopperShuffleTimer.reset();
          if (timeout(2.0)) {

            yield DeployState.HOPPER_SHUFFLING_OUT;
          }
        }
        yield currentState;
      }
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
      default -> {
        differentialMechanism.setControl(
            differentialPositionVoltageRequest
                .withAveragePosition(clamp(newState.getLength()))
                .withDifferentialPosition(0));
      }
    }
  }

  @Override
  protected void whileInState(DeployState state) {
    DogLog.log("Deploy/LeftMotor/Position", leftMotorPosition);
    DogLog.log("Deploy/RightMotor/Position", rightMotorPosition);
    DogLog.log("Deploy/GoalPosition", getState().getLength());
    DogLog.log("Deploy/DifferentialPosition", differentialMechanismPosition);
    DogLog.log("Deploy/AbleToHopperShuffle", hopperCapacityNotHigh);
    DogLog.log("Deploy/Capacity", hopperCapacity);
    DogLog.log("Hopper/RawDistance", hopperCANRangeDistance);
    DogLog.log("Hopper/FilteredDistance", filteredHopperCANRangeDistance);
    DogLog.log("Deploy/LeftMotor/StatorCurrent", leftStatorCurrent);
    DogLog.log("Deploy/LeftMotor/SupplyCurrent", leftSupplyCurrent);
    DogLog.log("Deploy/RightMotor/StatorCurrent", rightStatorCurrent);
    DogLog.log("Deploy/RightMotor/SupplyCurrent", rightSupplyCurrent);

    // TODO: Remove after bringup
    afterTransition(state);
  }

  public double getPosition() {
    return differentialMechanismPosition;
  }

  private boolean atGoal() {
    return switch (getState()) {
      case UNHOMED, HOME_INWARD, HOME_OUTWARD -> false;
      default ->
          MathUtil.isNear(
                  getState().getLength(), leftMotorPosition, DeployConfig.POSITION_TOLERANCE)
              && MathUtil.isNear(
                  getState().getLength(), rightMotorPosition, DeployConfig.POSITION_TOLERANCE);
    };
  }

  public boolean atGoal(double goalDistance) {
    return switch (getState()) {
      case UNHOMED, HOME_INWARD, HOME_OUTWARD -> false;
      default ->
          MathUtil.isNear(goalDistance, leftMotorPosition, DeployConfig.POSITION_TOLERANCE)
              && MathUtil.isNear(goalDistance, rightMotorPosition, DeployConfig.POSITION_TOLERANCE);
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

    hopperCANRangeDistance = Units.metersToInches(hopperCANRange.getDistance().getValueAsDouble());
    filteredHopperCANRangeDistance = hopperFilter.calculate(hopperCANRangeDistance);

    if (filteredHopperCANRangeDistance >= DeployConfig.HIGH_CAPACITY_THRESHOLD) {
      hopperCapacity = HopperCapacity.HIGH;
    } else if (filteredHopperCANRangeDistance >= DeployConfig.MEDIUM_CAPACITY_THRESHOLD) {
      hopperCapacity = HopperCapacity.MEDIUM;
    } else {
      hopperCapacity = HopperCapacity.LOW;
    }

    if (RobotBase.isSimulation()) {
      hopperCapacityNotHigh = true;
    } else {
      hopperCapacityNotHigh =
          !DSOptions.USE_CANRANGE.getAsBoolean() || hopperCapacity != HopperCapacity.HIGH;
    }
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();
    if (previousCanRangeDistance != hopperCANRangeDistance) {
      canRangeUpdateTimer.reset();
    }
    previousCanRangeDistance = hopperCANRangeDistance;

    if (canRangeUpdateTimer.hasElapsed(DeployConfig.NOT_UPDATING_TIMEOUT)) {
      DogLog.logFault("CANrange distance not updating", AlertType.kError);
    } else {
      DogLog.clearFault("CANrange distance not updating");
    }
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
}
