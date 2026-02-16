package frc.robot.deploy;

import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.team581.math.MathHelpers;
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
  private final CANrange hopperCANRange;
  private final LinearFilter hopperFilter = LinearFilter.movingAverage(5);
  private final CoastOut coastRequest = new CoastOut();
  private final MotionMagicVoltage positionVoltageRequest =
      new MotionMagicVoltage(0).withEnableFOC(false);

  private HopperCapacity hopperCapacity = HopperCapacity.LOW;
  private DeployState storedState = DeployState.UNHOMED;
  private double leftMotorPosition = 0.0;
  private double rightMotorPosition = 0.0;
  private double leftStatorCurrent = 0.0;
  private double rightStatorCurrent = 0.0;
  private double leftSupplyCurrent = 0.0;
  private double rightSupplyCurrent = 0.0;
  private double hopperCANRangeDistance = 0.0;
  private double previousCanRangeDistance = 0.0;
  private double filteredHopperCANRangeDistance;
  private boolean ableToHopperShuffle = false;

  private final Timer canRangeUpdateTimer = new Timer();

  public Deploy(TalonFX leftMotor, TalonFX rightMotor, CANrange hopperCANRange) {
    super(SubsystemPriority.DEPLOY, DeployState.UNHOMED);
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
    this.hopperCANRange = hopperCANRange;
    canRangeUpdateTimer.start();

    leftMotor.getConfigurator().apply(DeployConfig.LEFT_MOTOR_CONFIG);
    rightMotor.getConfigurator().apply(DeployConfig.RIGHT_MOTOR_CONFIG);
    hopperCANRange.getConfigurator().apply(DeployConfig.CAN_RANGE_CONFIG);

    TunablePid.register("Deploy/Left", leftMotor, DeployConfig.LEFT_MOTOR_CONFIG);
    TunablePid.register("Deploy/Right", rightMotor, DeployConfig.RIGHT_MOTOR_CONFIG);
  }

  public void intakeRequest() {
    switch (getState()) {
      case UNHOMED, HOME, CATCHUP_TO_LEFT, CATCHUP_TO_RIGHT -> {
        // Do nothing, we aren't homed or need to catchup
      }
      default -> setStateFromRequest(DeployState.INTAKE);
    }
  }

  public void stowRequest() {
    switch (getState()) {
      case UNHOMED, HOME, CATCHUP_TO_LEFT, CATCHUP_TO_RIGHT -> {
        // Do nothing, we aren't homed or need to catchup
      }
      default -> setStateFromRequest(DeployState.STOW);
    }
  }

  public void shuffleRequest() {
    switch (getState()) {
      case UNHOMED, HOME, CATCHUP_TO_LEFT, CATCHUP_TO_RIGHT -> {
        // Do nothing, we aren't homed or need to catchup
      }
      default -> {
        if (FeatureFlags.HOPPER_SHUFFLING.getAsBoolean()) {
          setStateFromRequest(DeployState.HOPPER_SHUFFLING);
        }
      }
    }
  }

  public void homingRequest() {
    setStateFromRequest(DeployState.HOME);
  }

  @Override
  protected void beforeTransition(DeployState oldState, DeployState newState) {
    if (newState == DeployState.CATCHUP_TO_LEFT || newState == DeployState.CATCHUP_TO_RIGHT) {
      storedState = getState();
    }
  }

  @Override
  protected DeployState getNextState(DeployState currentState) {
    return switch (currentState) {
      // Do nothing
      case UNHOMED -> currentState;

      case HOME -> {
        if (leftMotor.getStatorCurrent().getValueAsDouble() > DeployConfig.HOMING_CURRENT
            && rightMotor.getStatorCurrent().getValueAsDouble() > DeployConfig.HOMING_CURRENT) {
          leftMotor.setPosition(DeployConfig.HOMING_END_POSITION);
          rightMotor.setPosition(DeployConfig.HOMING_END_POSITION);
          yield DeployState.INTAKE;
        } else {
          yield DeployState.HOME;
        }
      }

      case CATCHUP_TO_LEFT, CATCHUP_TO_RIGHT -> {
        if (MathUtil.isNear(
            leftMotorPosition, rightMotorPosition, DeployConfig.POSITION_TOLERANCE)) {
          yield storedState;
        }
        yield currentState;
      }

      case INTAKE, STOW, HOPPER_SHUFFLING -> {
        if (!MathUtil.isNear(leftMotorPosition, rightMotorPosition, 1)) {
          DogLog.logFault("DEPLOY MOTORS NOT ALIGNED", AlertType.kError);
          if (leftMotorPosition > rightMotorPosition) {
            yield DeployState.CATCHUP_TO_LEFT;
          }

          yield DeployState.CATCHUP_TO_RIGHT;
        } else {
          DogLog.clearFault("DEPLOY MOTORS NOT ALIGNED");
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
      case HOME -> {
        leftMotor.setVoltage(DeployConfig.HOMING_VOLTAGE);
        rightMotor.setVoltage(DeployConfig.HOMING_VOLTAGE);
      }
      case CATCHUP_TO_LEFT -> {
        leftMotor.disable();
        rightMotor.setControl(positionVoltageRequest.withPosition(leftMotorPosition));
      }
      case CATCHUP_TO_RIGHT -> {
        leftMotor.setControl(positionVoltageRequest.withPosition(rightMotorPosition));
        rightMotor.disable();
      }
      default -> {
        leftMotor.setControl(positionVoltageRequest.withPosition(clamp(newState.getLength())));
        rightMotor.setControl(positionVoltageRequest.withPosition(clamp(newState.getLength())));
      }
    }
  }

  @Override
  protected void whileInState(DeployState state) {
    if (DriverStation.isDisabled()) {
      leftMotor.setControl(coastRequest);
      rightMotor.setControl(coastRequest);
    } else {
      switch (state) {
        case HOME -> {
          leftMotor.setVoltage(DeployConfig.HOMING_VOLTAGE);
          rightMotor.setVoltage(DeployConfig.HOMING_VOLTAGE);
        }
        case CATCHUP_TO_LEFT -> {
          leftMotor.disable();
          rightMotor.setControl(positionVoltageRequest.withPosition(leftMotorPosition));
        }
        case CATCHUP_TO_RIGHT -> {
          leftMotor.setControl(positionVoltageRequest.withPosition(rightMotorPosition));
          rightMotor.disable();
        }
      }
      if (FeatureFlags.HOPPER_SHUFFLING.getAsBoolean()
          && state == DeployState.HOPPER_SHUFFLING
          && ableToHopperShuffle) {
        if (atGoal(DeployState.HOPPER_SHUFFLING.getLength())) {
          leftMotor.setControl(
              positionVoltageRequest.withPosition(
                  clamp(
                      DeployState.HOPPER_SHUFFLING.getLength()
                          - DeployConfig.HOPPER_SHUFFLE_DISTANCE)));
          rightMotor.setControl(
              positionVoltageRequest.withPosition(
                  clamp(
                      DeployState.HOPPER_SHUFFLING.getLength()
                          - DeployConfig.HOPPER_SHUFFLE_DISTANCE)));
        } else if (atGoal(
            DeployState.HOPPER_SHUFFLING.getLength() - DeployConfig.HOPPER_SHUFFLE_DISTANCE)) {
          leftMotor.setControl(
              positionVoltageRequest.withPosition(clamp(DeployState.HOPPER_SHUFFLING.getLength())));
          rightMotor.setControl(
              positionVoltageRequest.withPosition(clamp(DeployState.HOPPER_SHUFFLING.getLength())));
        }
      }
    }

    DogLog.log("Deploy/LeftMotor/Position", leftMotorPosition);
    DogLog.log("Deploy/RightMotor/Position", rightMotorPosition);
    DogLog.log("Deploy/GoalPosition", getState().getLength());
    DogLog.log("Deploy/AveragePosition", getPosition());
    DogLog.log("Deploy/AbleToHopperShuffle", ableToHopperShuffle);
    DogLog.log("Deploy/StoredState", storedState.name());
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
    return MathHelpers.average(leftMotorPosition, rightMotorPosition);
  }

  private boolean atGoal(double goalDistance) {
    return MathUtil.isNear(goalDistance, leftMotorPosition, DeployConfig.POSITION_TOLERANCE)
        && MathUtil.isNear(goalDistance, rightMotorPosition, DeployConfig.POSITION_TOLERANCE);
  }

  @Override
  protected void collectInputs() {
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
      ableToHopperShuffle = true;
    } else {
      ableToHopperShuffle =
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
  public void disabledPeriodic() {
    if (DriverStation.isDisabled()) {
      leftMotor.setControl(new CoastOut());
      rightMotor.setControl(new CoastOut());
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

    if (getState() == DeployState.HOME) {
      leftMotor.setPosition(DeployConfig.HOMING_END_POSITION);
      rightMotor.setPosition(DeployConfig.HOMING_END_POSITION);
      setStateFromRequest(DeployState.INTAKE);
    }

    deploySimulation.update();
  }
}
