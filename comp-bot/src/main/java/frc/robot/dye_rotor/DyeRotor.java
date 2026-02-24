package frc.robot.dye_rotor;

import com.ctre.phoenix6.controls.VelocityVoltage;
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
import frc.robot.util.scheduling.SubsystemPriority;

public class DyeRotor extends StateMachineSubsystem<DyeRotorState> {
  private final LinearFilter currentFilter = LinearFilter.movingAverage(5);

  private final TalonFX rotorMotor;
  private final TalonFX horizontalMotor;
  private final TalonFX verticalMotor;

  private final VelocityVoltage rotorVelocityRequest = new VelocityVoltage(0).withEnableFOC(false);

  private double horizontalRawCurrent = 0.0;
  private double verticalRawCurrent = 0.0;
  private double rotorRawCurrent = 0.0;
  private double rotorFilteredCurrent = 0.0;

  private double rotorMotorRpm = 0.0;
  private double rotorAngle = 0.0;
  private double horizontalMotorRpm = 0.0;
  private boolean isShooting = false;
  private boolean isShootingDebounced = false;

  private double scoreDistance = 0;
  private double feedDistance = 0;

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
    if (getState() != DyeRotorState.UNHOMED) {
      setStateFromRequest(DyeRotorState.SCORE);
    }
  }

  public void feedRequest(double distance) {
        feedDistance = distance;

    if (getState() != DyeRotorState.UNHOMED) {
      setStateFromRequest(DyeRotorState.FEED);
    }
  }

  public void scoreCleanupRequest(double distance) {
    scoreDistance = distance;
    if (getState() != DyeRotorState.UNHOMED) {
      setStateFromRequest(DyeRotorState.SCORE_CLEANUP_INTAKE_SCAN);
    }
  }

  public void feedCleanupRequest(double distance) {
    feedDistance = distance;
    if (getState() != DyeRotorState.UNHOMED) {
      setStateFromRequest(DyeRotorState.FEED_CLEANUP_INTAKE_SCAN);
    }
  }

  public void unjamRequest() {
    if (getState() != DyeRotorState.UNHOMED) {
      setStateFromRequest(DyeRotorState.UNJAM);
    }
  }

  public void idleRequest() {
    if (getState() != DyeRotorState.UNHOMED) {
      setStateFromRequest(DyeRotorState.RESET_TO_IDLE);
    }
  }

  private boolean nearIdlePosition() {
    return MathUtil.isNear(DyeRotorState.IDLE.rotorPosition, rotorAngle, 45, -180, 180);
  }

  @Override
  protected DyeRotorState getNextState(DyeRotorState currentState) {
    return switch (currentState) {
      case UNHOMED -> {
        if (rotorMotor.isAlive() && rotorMotor.isConnected()) {
          rotorMotor.setPosition(Units.degreesToRotations(DyeRotorConfig.HOMING_END_POSITION));
          yield DyeRotorState.RESET_TO_IDLE;
        }
        yield currentState;
      }
      case RESET_TO_IDLE -> {
        if (nearIdlePosition()) {
          yield DyeRotorState.IDLE;
        }
        yield currentState;
      }
      case IDLE -> {
        if (!nearIdlePosition()) {
          yield DyeRotorState.RESET_TO_IDLE;
        }
        yield currentState;
      }
      case SCORE_CLEANUP_INTAKE_SCAN -> {
        if (rotorAngle >= DyeRotorState.SCORE_CLEANUP_WHIP_AROUND.rotorPosition
            || rotorAngle <= DyeRotorState.SCORE_CLEANUP_INTAKE_SCAN.rotorPosition) {
          yield DyeRotorState.SCORE_CLEANUP_WHIP_AROUND;
        } else {
          yield currentState;
        }
      }
      case SCORE_CLEANUP_WHIP_AROUND -> {
        if (rotorAngle >= DyeRotorState.SCORE_CLEANUP_INTAKE_SCAN.rotorPosition
            && rotorAngle < DyeRotorState.SCORE_CLEANUP_WHIP_AROUND.rotorPosition) {
          yield DyeRotorState.SCORE_CLEANUP_INTAKE_SCAN;
        } else {
          yield currentState;
        }
      }
         case FEED_CLEANUP_INTAKE_SCAN -> {
        if (rotorAngle >= DyeRotorState.SCORE_CLEANUP_WHIP_AROUND.rotorPosition
            || rotorAngle <= DyeRotorState.SCORE_CLEANUP_INTAKE_SCAN.rotorPosition) {
          yield DyeRotorState.FEED_CLEANUP_WHIP_AROUND;
        } else {
          yield currentState;
        }
      }
      case FEED_CLEANUP_WHIP_AROUND -> {
        if (rotorAngle >= DyeRotorState.SCORE_CLEANUP_INTAKE_SCAN.rotorPosition
            && rotorAngle < DyeRotorState.SCORE_CLEANUP_WHIP_AROUND.rotorPosition) {
          yield DyeRotorState.FEED_CLEANUP_INTAKE_SCAN;
        } else {
          yield currentState;
        }
      }
      default -> currentState;
    };
  }

  @Override
  protected void whileInState(DyeRotorState currentState) {

    switch (currentState) {
      case SCORE, SCORE_CLEANUP_INTAKE_SCAN, SCORE_CLEANUP_WHIP_AROUND-> {
        rotorMotor.setControl(rotorVelocityRequest.withVelocity(currentState.getRotorRPM(DyeRotorConfig.DISTANCE_TO_SCORE_BPS.get(scoreDistance)) / 60.0));
        horizontalMotor.setVoltage(currentState.getHorizontalVoltage());
        verticalMotor.setVoltage(currentState.getVerticalVoltage());
      }
      case FEED, FEED_CLEANUP_INTAKE_SCAN, FEED_CLEANUP_WHIP_AROUND-> {
        rotorMotor.setControl(rotorVelocityRequest.withVelocity(currentState.getRotorRPM(DyeRotorConfig.DISTANCE_TO_FEED_BPS.get(feedDistance)) / 60.0));
        horizontalMotor.setVoltage(currentState.getHorizontalVoltage());
        verticalMotor.setVoltage(currentState.getVerticalVoltage());
      }
      default -> {
        rotorMotor.setControl(rotorVelocityRequest.withVelocity(currentState.getRotorRPM() / 60.0));
        horizontalMotor.setVoltage(currentState.getHorizontalVoltage());
        verticalMotor.setVoltage(currentState.getVerticalVoltage());
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
  }

  @Override
  protected void collectInputs() {
    verticalRawCurrent = verticalMotor.getStatorCurrent().getValueAsDouble();
    horizontalRawCurrent = horizontalMotor.getStatorCurrent().getValueAsDouble();
    rotorRawCurrent = rotorMotor.getStatorCurrent().getValueAsDouble();
    rotorFilteredCurrent = currentFilter.calculate(rotorRawCurrent);

    rotorMotorRpm = rotorMotor.getVelocity().getValueAsDouble() * 60.0;
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
}
