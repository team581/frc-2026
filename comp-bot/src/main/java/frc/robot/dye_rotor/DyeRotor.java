package frc.robot.dye_rotor;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
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
  private final VelocityVoltage horizontalVelocityRequest =
      new VelocityVoltage(0).withEnableFOC(false);

  private double rotorRawCurrent = 0.0;
  private double rotorFilteredCurrent = 0.0;

  private double rotorMotorRpm = 0.0;
  private double rotorAngle = 0.0;
  private double horizontalMotorRpm = 0.0;
  private boolean isShooting = false;
  private boolean isShootingDebounced = false;

  public DyeRotor(TalonFX rotorMotor, TalonFX horizontalMotor, TalonFX verticalMotor) {
    super(SubsystemPriority.DYE_ROTOR, DyeRotorState.IDLE);

    rotorMotor.getConfigurator().apply(DyeRotorConfig.ROTOR_MOTOR_CONFIG);
    horizontalMotor.getConfigurator().apply(DyeRotorConfig.HORIZONTAL_MOTOR_CONFIG);
    verticalMotor.getConfigurator().apply(DyeRotorConfig.VERTICAL_MOTOR_CONFIG);

    TunablePid.register("DyeRotor/Rotor", rotorMotor, DyeRotorConfig.ROTOR_MOTOR_CONFIG);
    TunablePid.register(
        "DyeRotor/Horizontal", horizontalMotor, DyeRotorConfig.HORIZONTAL_MOTOR_CONFIG);
    TunablePid.register("DyeRotor/Vertical", verticalMotor, DyeRotorConfig.VERTICAL_MOTOR_CONFIG);

    this.rotorMotor = rotorMotor;
    this.horizontalMotor = horizontalMotor;
    this.verticalMotor = verticalMotor;
  }

  public void shootRequest() {
    setStateFromRequest(DyeRotorState.SHOOTING);
  }

  public void warmupRequest() {
    setStateFromRequest(DyeRotorState.WARMUP);
  }

  public void unjamRequest() {
    setStateFromRequest(DyeRotorState.UNJAM);
  }

  public void idleRequest() {
    setStateFromRequest(DyeRotorState.IDLE);
  }

  @Override
  protected void whileInState(DyeRotorState state) {
    DogLog.log("DyeRotor/Rotor/RPM", rotorMotorRpm);
    DogLog.log("DyeRotor/Rotor/GoalRPM", state.rotorRPM);
    DogLog.log("DyeRotor/Rotor/Angle", rotorAngle);
    DogLog.log("DyeRotor/Rotor/Voltage", rotorMotor.getMotorVoltage().getValueAsDouble());
    DogLog.log("DyeRotor/Horizontal/RPM", horizontalMotorRpm);
    DogLog.log("DyeRotor/Horizontal/GoalRPM", state.horizontalRPM);
    DogLog.log("DyeRotor/Horizontal/Voltage", horizontalMotor.getMotorVoltage().getValueAsDouble());
    DogLog.log("DyeRotor/Vertical/GoalVoltage", state.verticalVoltage);
    DogLog.log("DyeRotor/Vertical/Voltage", verticalMotor.getMotorVoltage().getValueAsDouble());
    DogLog.log("DyeRotor/AtGoal", atGoal());
  }

  @Override
  protected void afterTransition(DyeRotorState newState) {
    rotorMotor.setControl(rotorVelocityRequest.withVelocity(newState.rotorRPM / 60.0));
    horizontalMotor.setControl(
        horizontalVelocityRequest.withVelocity(newState.horizontalRPM / 60.0));
    verticalMotor.setVoltage(newState.verticalVoltage);
  }

  @Override
  protected void collectInputs() {
    rotorRawCurrent = rotorMotor.getStatorCurrent().getValueAsDouble();
    rotorFilteredCurrent = currentFilter.calculate(rotorRawCurrent);

    rotorMotorRpm = rotorMotor.getVelocity().getValueAsDouble() * 60.0;
    rotorAngle = Units.rotationsToDegrees(rotorMotor.getPosition().getValueAsDouble());
    horizontalMotorRpm = horizontalMotor.getVelocity().getValueAsDouble() * 60.0;

    isShooting = horizontalMotorRpm < DyeRotorConfig.RPM_TOLERANCE_SHOOTING;
    isShootingDebounced = DyeRotorConfig.debouncer.calculate(isShooting);
  }

  public boolean atGoal() {
    return switch (getState()) {
      case IDLE -> true;
      case UNJAM -> timeout(1) || !isJammed();
      case SHOOTING -> true;
      case WARMUP ->
          MathUtil.isNear(
              DyeRotorState.WARMUP.horizontalRPM,
              horizontalMotorRpm,
              DyeRotorConfig.RPM_TOLERANCE_HORIZONTAL);
    };
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
