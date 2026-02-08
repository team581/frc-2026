package frc.robot.deploy;

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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.config.FeatureFlags;
import frc.robot.util.scheduling.SubsystemPriority;

public class Deploy extends StateMachineSubsystem<DeployState> {
  private final TalonFX leftMotor;
  private final TalonFX rightMotor;
  private final CANrange hopperCANRange;
  private final MotionMagicVoltage positionVoltageRequest =
      new MotionMagicVoltage(0).withEnableFOC(false);
  private DeployState storedState = DeployState.UNHOMED;
  private double leftMotorPosition = 0.0;
  private double rightMotorPosition = 0.0;
  private double hopperCANRangeDistance = 0.0;
  private boolean ableToHopperShuffle = false;

  public Deploy(TalonFX leftMotor, TalonFX rightMotor, CANrange hopperCANRange) {
    super(SubsystemPriority.DEPLOY, DeployState.UNHOMED);
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
    this.hopperCANRange = hopperCANRange;

    leftMotor.getConfigurator().apply(DeployConfig.LEFT_MOTOR_CONFIG);
    rightMotor.getConfigurator().apply(DeployConfig.RIGHT_MOTOR_CONFIG);
    hopperCANRange.getConfigurator().apply(DeployConfig.CAN_RANGE_CONFIG);

    TunablePid.register("Deploy/Left", leftMotor, DeployConfig.LEFT_MOTOR_CONFIG);
    TunablePid.register("Deploy/Right", rightMotor, DeployConfig.RIGHT_MOTOR_CONFIG);
  }

  public void intakeRequest() {
    switch (getState()) {
      case UNHOMED, HOMING, CATCHUP_TO_LEFT, CATCHUP_TO_RIGHT -> {
        // Do nothing, we aren't homed or need to catchup
      }
      default -> setStateFromRequest(DeployState.INTAKE);
    }
  }

  public void stowRequest() {
    switch (getState()) {
      case UNHOMED, HOMING, CATCHUP_TO_LEFT, CATCHUP_TO_RIGHT -> {
        // Do nothing, we aren't homed or need to catchup
      }
      default -> setStateFromRequest(DeployState.STOWED);
    }
  }

  public void shootingRequest() {
    switch (getState()) {
      case UNHOMED, HOMING, CATCHUP_TO_LEFT, CATCHUP_TO_RIGHT -> {
        // Do nothing, we aren't homed or need to catchup
      }
      default -> setStateFromRequest(DeployState.SHOOTING);
    }
  }

  public void homingRequest() {
    setStateFromRequest(DeployState.HOMING);
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
      case HOMING -> {
        if (leftMotor.getStatorCurrent().getValueAsDouble() > DeployConfig.HOMING_CURRENT
            && rightMotor.getStatorCurrent().getValueAsDouble() > DeployConfig.HOMING_CURRENT) {
          leftMotor.setPosition(DeployConfig.HOMING_END_POSITION);
          rightMotor.setPosition(DeployConfig.HOMING_END_POSITION);
          yield DeployState.INTAKE;
        } else {
          yield DeployState.HOMING;
        }
      }
      case CATCHUP_TO_LEFT, CATCHUP_TO_RIGHT -> {
        if (MathUtil.isNear(
            leftMotorPosition, rightMotorPosition, DeployConfig.POSITION_TOLERANCE)) {
          yield storedState;
        }
        yield currentState;
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
      case UNHOMED -> {
        leftMotor.disable();
        rightMotor.disable();
      }
      case HOMING -> {
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
      case SHOOTING -> {
        leftMotor.setControl(
            positionVoltageRequest.withPosition(clamp(DeployState.SHOOTING.getLength())));
        rightMotor.setControl(
            positionVoltageRequest.withPosition(clamp(DeployState.SHOOTING.getLength())));
      }
      default -> {
        leftMotor.setControl(positionVoltageRequest.withPosition(clamp(newState.getLength())));
        rightMotor.setControl(positionVoltageRequest.withPosition(clamp(newState.getLength())));
      }
    }
  }

  @Override
  protected void whileInState(DeployState state) {
    if (FeatureFlags.HOPPER_SHUFFLING.getAsBoolean()
        && state == DeployState.SHOOTING
        && ableToHopperShuffle) {
      if (atGoal(DeployState.SHOOTING.getLength())) {
        leftMotor.setControl(
            positionVoltageRequest.withPosition(clamp(DeployState.INTAKE.getLength())));
        rightMotor.setControl(
            positionVoltageRequest.withPosition(clamp(DeployState.INTAKE.getLength())));
      } else if (atGoal(DeployState.INTAKE.getLength())) {
        leftMotor.setControl(
            positionVoltageRequest.withPosition(clamp(DeployState.SHOOTING.getLength())));
        rightMotor.setControl(
            positionVoltageRequest.withPosition(clamp(DeployState.SHOOTING.getLength())));
      }
    }

    if (!MathUtil.isNear(leftMotorPosition, rightMotorPosition, 1)) {
      DogLog.logFault("DEPLOY MOTORS NOT ALIGNED", AlertType.kError);
      if (leftMotorPosition > rightMotorPosition) {
        setStateFromRequest(DeployState.CATCHUP_TO_LEFT);
      } else {
        setStateFromRequest(DeployState.CATCHUP_TO_RIGHT);
      }
    }
    DogLog.clearFault("DEPLOY MOTORS NOT ALIGNED");

    DogLog.log("Deploy/LeftMotor/Position", leftMotorPosition);
    DogLog.log("Deploy/RightMotor/Position", rightMotorPosition);
    DogLog.log("Deploy/AveragePosition", getPosition());
    DogLog.log("Deploy/HopperCANRangeDistance", hopperCANRangeDistance);
    DogLog.log("Deploy/AbleToHopperShuffle", ableToHopperShuffle);
    DogLog.log("Deploy/StoredState", storedState.name());
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
    hopperCANRangeDistance = Units.metersToInches(hopperCANRange.getDistance().getValueAsDouble());
    if (RobotBase.isSimulation()) {
      ableToHopperShuffle = true;
    } else {
      ableToHopperShuffle = hopperCANRangeDistance < DeployConfig.CAPACITY_DISTANCE_THRESHOLD;
    }
  }

  @Override
  public void simulationPeriodic() {
    var deploySimulation =
        SimKit.positionMechanism(
            "Deploy/Left",
            mechanism ->
                mechanism
                    .addMotor(leftMotor, ChassisReference.Clockwise_Positive)
                    .addMotor(rightMotor, ChassisReference.CounterClockwise_Positive)
                    .withMinPosition(DeployConfig.MIN_LENGTH)
                    .withMaxPosition(DeployConfig.MAX_LENGTH));

    if (getState() == DeployState.HOMING) {
      leftMotor.setPosition(DeployConfig.HOMING_END_POSITION);
      rightMotor.setPosition(DeployConfig.HOMING_END_POSITION);
      setStateFromRequest(DeployState.INTAKE);
    }

    deploySimulation.update();
  }
}
