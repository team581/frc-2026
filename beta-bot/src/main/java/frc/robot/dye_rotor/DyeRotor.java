package frc.robot.dye_rotor;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.team581.math.MathHelpers;
import com.team581.mechanisms.PowerManaged;
import com.team581.mechanisms.VelocityDetector;
import com.team581.simkit.SimKit;
import com.team581.util.state_machines.StateMachineSubsystem;
import com.team581.util.tuning.TunablePid;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.util.Units;
import frc.robot.util.scheduling.SubsystemPriority;

public class DyeRotor extends StateMachineSubsystem<DyeRotorState> implements PowerManaged {
  private final LinearFilter currentFilter = LinearFilter.movingAverage(10);

  private final TalonFX rotorMotor;
  private final TalonFX horizontalMotor;
  private final TalonFX verticalMotor;

  private final VelocityVoltage rotorVelocityRequest = new VelocityVoltage(0).withEnableFOC(false);
  private final VoltageOut horizontalVoltageRequest = new VoltageOut(0).withEnableFOC(false);
  private final VoltageOut verticalVoltageRequest = new VoltageOut(0).withEnableFOC(false);

  private final LinearFilter velocityAverage = LinearFilter.movingAverage(8);
  private double averageRotorRpm = 0.0;
  private final VelocityDetector autoGpDetection =
      new VelocityDetector(DyeRotorConfig.GP_DETECT_VELOCITY_THRESHOLD);

  private double horizontalRawCurrent = 0.0;
  private double verticalRawCurrent = 0.0;
  private double rotorRawCurrent = 0.0;
  private double rotorFilteredCurrent = 0.0;

  private double rotorMotorRpm = 0.0;
  private double rotorAngle = 0.0;
  private double horizontalMotorRpm = 0.0;
  private boolean isShooting = false;
  private boolean isShootingDebounced = false;

  private boolean useFullSpeed = false;

  private double scoreDistance = 0;
  private double feedDistance = 0;

  private DyeRotorState beforeUnjamState = DyeRotorState.IDLE;

  public DyeRotor(TalonFX rotorMotor, TalonFX horizontalMotor, TalonFX verticalMotor) {
    super(SubsystemPriority.DYE_ROTOR, DyeRotorState.UNHOMED);

    rotorMotor.getConfigurator().apply(DyeRotorConfig.ROTOR_MOTOR_CONFIG);
    horizontalMotor.getConfigurator().apply(DyeRotorConfig.HORIZONTAL_MOTOR_CONFIG);
    verticalMotor.getConfigurator().apply(DyeRotorConfig.VERTICAL_MOTOR_CONFIG);

    TunablePid.register("DyeRotor/Rotor", rotorMotor, DyeRotorConfig.ROTOR_MOTOR_CONFIG);

    this.rotorMotor = rotorMotor;
    this.horizontalMotor = horizontalMotor;
    this.verticalMotor = verticalMotor;
  }

  public void scoreRequest(double distance) {
    scoreDistance = distance;
    switch (getState()) {
      case UNJAM, UNHOMED -> {}
      default -> setStateFromRequest(DyeRotorState.SCORE);
    }
  }

  public void scoreSlowRequest() {
    if (getState() != DyeRotorState.UNHOMED) {
      setStateFromRequest(DyeRotorState.SCORE_SLOW);
    }
  }

  public void feedRequest(double distance) {
    feedDistance = distance;

    switch (getState()) {
      case UNJAM, UNHOMED -> {}
      default -> setStateFromRequest(DyeRotorState.FEED);
    }
  }

  public void unjamRequest() {
    if (getState() != DyeRotorState.UNHOMED) {
      beforeUnjamState = getState();
      setStateFromRequest(DyeRotorState.UNJAM);
    }
  }

  public void idleRequest() {
    if (getState() != DyeRotorState.UNHOMED) {
      setStateFromRequest(DyeRotorState.IDLE);
    }
  }

  public void resetToIdleRequest() {
    if (getState() != DyeRotorState.UNHOMED) {
      setStateFromRequest(DyeRotorState.RESET_TO_IDLE);
    }
  }

  public void setUseFullSpeed(boolean useFullSpeed) {
    this.useFullSpeed = useFullSpeed;
  }

  public boolean isReset() {
    if (getState() == DyeRotorState.IDLE && Math.abs(rotorMotorRpm) < 1e-2) {
      return true;
    }
    return false;
  }

  private boolean nearIdlePosition() {
    return MathUtil.isNear(DyeRotorState.IDLE.rotorPosition, rotorAngle, 30, -180, 180);
  }

  @Override
  protected DyeRotorState getNextState(DyeRotorState currentState) {
    return switch (currentState) {
      case UNHOMED -> {
        if (rotorMotor.isAlive() && rotorMotor.isConnected()) {
          rotorMotor.setPosition(Units.degreesToRotations(DyeRotorConfig.HOMING_END_POSITION));
          yield DyeRotorState.IDLE;
        }
        yield currentState;
      }
      case RESET_TO_IDLE -> {
        if (nearIdlePosition()) {
          yield DyeRotorState.IDLE;
        }
        yield currentState;
      }
      case UNJAM -> {
        if (timeout(DyeRotorConfig.UNJAM_TIMEOUT.get())) {
          yield beforeUnjamState;
        }
        yield currentState;
      }

      case SCORE, SCORE_SLOW, FEED -> {
        if (isJammed()) {
          beforeUnjamState = currentState;
          yield DyeRotorState.UNJAM;
        }
        yield currentState;
      }
      default -> currentState;
    };
  }

  @Override
  protected void whileInState(DyeRotorState currentState) {

    switch (currentState) {
      case SCORE, SCORE_SLOW -> {
        var wantedRPM =
            useFullSpeed
                ? DyeRotorState.bpsToRpm(DyeRotorConfig.FULL_SPEED_BPS)
                : currentState.getRotorRPM(DyeRotorConfig.DISTANCE_TO_SCORE_BPS.get(scoreDistance));
        rotorMotor.setControl(rotorVelocityRequest.withVelocity(wantedRPM / 60.0));
        horizontalMotor.setControl(
            horizontalVoltageRequest.withOutput(currentState.getHorizontalVoltage()));
        verticalMotor.setControl(
            verticalVoltageRequest.withOutput(currentState.getVerticalVoltage()));
      }
      case FEED -> {
        rotorMotor.setControl(
            rotorVelocityRequest.withVelocity(
                currentState.getRotorRPM(DyeRotorConfig.DISTANCE_TO_FEED_BPS.get(feedDistance))
                    / 60.0));
        horizontalMotor.setControl(
            horizontalVoltageRequest.withOutput(currentState.getHorizontalVoltage()));
        verticalMotor.setControl(
            verticalVoltageRequest.withOutput(currentState.getVerticalVoltage()));
      }
      default -> {
        rotorMotor.setControl(rotorVelocityRequest.withVelocity(currentState.getRotorRPM() / 60.0));
        horizontalMotor.setControl(
            horizontalVoltageRequest.withOutput(currentState.getHorizontalVoltage()));
        verticalMotor.setControl(
            verticalVoltageRequest.withOutput(currentState.getVerticalVoltage()));
      }
    }

    DogLog.log("DyeRotor/Rotor/RPM", rotorMotorRpm);
    DogLog.log("DyeRotor/Rotor/GoalRPM", currentState.rotorRPM);
    DogLog.log("DyeRotor/Rotor/Angle", rotorAngle);
    DogLog.log("DyeRotor/Rotor/Voltage", rotorMotor.getMotorVoltage().getValueAsDouble());
    DogLog.log("DyeRotor/Rotor/StatorCurrent", rotorRawCurrent);
    DogLog.log("DyeRotor/Horizontal/RPM", horizontalMotorRpm);
    DogLog.log("DyeRotor/Horizontal/GoalVoltage", currentState.getHorizontalVoltage());
    DogLog.log("DyeRotor/Horizontal/Voltage", horizontalMotor.getMotorVoltage().getValueAsDouble());
    DogLog.log("DyeRotor/Horizontal/StatorCurrent", horizontalRawCurrent);
    DogLog.log("DyeRotor/Vertical/GoalVoltage", currentState.getVerticalVoltage());
    DogLog.log("DyeRotor/Vertical/Voltage", verticalMotor.getMotorVoltage().getValueAsDouble());
    DogLog.log("DyeRotor/Vertical/Velocity", verticalMotor.getVelocity().getValueAsDouble());
    DogLog.log("DyeRotor/Vertical/StatorCurrent", verticalRawCurrent);
    DogLog.log("DyeRotor/IsJammed", isJammed());
    DogLog.log("DyeRotor/UseFullSpeed", useFullSpeed);
  }

  @Override
  protected void collectInputs() {
    verticalRawCurrent = verticalMotor.getStatorCurrent().getValueAsDouble();
    horizontalRawCurrent = horizontalMotor.getStatorCurrent().getValueAsDouble();
    rotorRawCurrent = rotorMotor.getStatorCurrent().getValueAsDouble();
    rotorFilteredCurrent = currentFilter.calculate(rotorRawCurrent);

    rotorMotorRpm = rotorMotor.getVelocity().getValueAsDouble() * 60.0;
    averageRotorRpm = velocityAverage.calculate(rotorMotorRpm);
    rotorAngle =
        MathHelpers.angleModulus(
            Units.rotationsToDegrees(rotorMotor.getPosition().getValueAsDouble()));
    horizontalMotorRpm = horizontalMotor.getVelocity().getValueAsDouble() * 60.0;

    isShooting = horizontalMotorRpm < DyeRotorConfig.RPM_TOLERANCE_SHOOTING;
    isShootingDebounced = DyeRotorConfig.IS_SHOOTING_DEBOUNCER.calculate(isShooting);
  }

  public boolean isJammed() {
    return rotorFilteredCurrent > DyeRotorConfig.JAM_CURRENT_THRESHOLD.getAsDouble();
  }

  public boolean isShooting() {
    if (isShootingDebounced) {
      return true;
    }
    return false;
  }

  public double getAngle() {
    return rotorAngle;
  }

  public boolean velocityDetectsGp() {
    DogLog.log("RobotManager/gpDetection/AverageRPM", averageRotorRpm);
    return !autoGpDetection.hasGamePiece(
        averageRotorRpm, DyeRotorConfig.GP_DETECT_VELOCITY_THRESHOLD);
  }

  @Override
  public void simulationPeriodic() {
    var rotorSimulation =
        SimKit.velocityMechanism(
            "DyeRotor/Rotor",
            (mechanism) -> mechanism.addMotor(rotorMotor, ChassisReference.Clockwise_Positive));
    var horizontalSimulation =
        SimKit.velocityMechanism(
            "DyeRotor/Horizontal",
            (mechanism) ->
                mechanism.addMotor(horizontalMotor, ChassisReference.CounterClockwise_Positive));
    var verticalSimulation =
        SimKit.velocityMechanism(
            "DyeRotor/Vertical",
            (mechanism) ->
                mechanism.addMotor(verticalMotor, ChassisReference.CounterClockwise_Positive));

    rotorSimulation.update();
    horizontalSimulation.update();
    verticalSimulation.update();
  }

  @Override
  public void applyCurrentLimits(double supplyCurrentLimit) {
    rotorMotor
        .getConfigurator()
        .apply(
            DyeRotorConfig.ROTOR_MOTOR_CONFIG.CurrentLimits.withSupplyCurrentLimit(
                supplyCurrentLimit));
    // horizontalMotor
    //     .getConfigurator()
    //     .apply(
    //         DyeRotorConfig.HORIZONTAL_MOTOR_CONFIG.CurrentLimits.withSupplyCurrentLimit(
    //             supplyCurrentLimit));
    // verticalMotor
    //     .getConfigurator()
    //     .apply(
    //         DyeRotorConfig.VERTICAL_MOTOR_CONFIG.CurrentLimits.withSupplyCurrentLimit(
    //             supplyCurrentLimit));
  }
}
