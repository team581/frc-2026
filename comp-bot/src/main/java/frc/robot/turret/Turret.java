package frc.robot.turret;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team581.math.MathHelpers;
import com.team581.simkit.SimKit;
import com.team581.util.state_machines.StateMachineSubsystem;
import com.team581.util.tuning.TunablePid;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.vision.Vision;

public class Turret extends StateMachineSubsystem<TurretState> {
  private final TalonFX motor;
  private double currentAngle = 0.0;
  private double goalAngle = 0.0;

  private final PositionVoltage positionRequest = new PositionVoltage(0.0).withEnableFOC(false);

  private final Vision vision;

  public Turret(TalonFX motor, Vision vision) {
    super(SubsystemPriority.TURRET, TurretState.UNHOMED);
    this.vision = vision;

    TunablePid.register("Turret", motor, TurretConfig.MOTOR_CONFIG);

    this.motor = motor;
  }

  @Override
  protected void collectInputs() {
    currentAngle = Units.rotationsToDegrees(motor.getPosition().getValueAsDouble());
    DogLog.log("Turret/Angle", currentAngle);

    // Predict the turret's current angle to account for sensor latency
    double latencyCompensatedAngle =
        Units.rotationsToDegrees(
            BaseStatusSignal.getLatencyCompensatedValueAsDouble(
                motor.getPosition(), motor.getVelocity()));
    DogLog.log("Turret/LatencyCompensatedAngle", latencyCompensatedAngle);

    // Add the predicted angle to the vision buffer at the current timestamp
    vision.addTurretObservation(
        Timer.getFPGATimestamp(), latencyCompensatedAngle, getVelocityDegreesPerSecond());
  }

  @Override
  protected void whileInState(TurretState currentState) {
    switch (currentState) {
      case UNHOMED -> {
        motor.disable();
      }
      case SCORE, FEED -> {
        motor.setControl(
            positionRequest.withPosition(
                Units.degreesToRotations(
                    TurretCalculator.getOptimalAngle(goalAngle, currentAngle))));
      }
      case IDLE_SCORE, IDLE_FEED -> {
        motor.setControl(
            positionRequest.withPosition(
                Units.degreesToRotations(
                    TurretCalculator.getSmartUnwrapAngle(goalAngle, currentAngle))));
      }
      default -> {}
    }
  }

  public void setState(TurretState newState) {
    switch (getState()) {
      case UNHOMED -> {}
      default -> {
        setStateFromRequest(newState);
      }
    }
  }

  public boolean goalOutOfBounds() {
    return goalAngle > (TurretConfig.MAX_ANGLE - TurretConfig.OUT_OF_BOUNDS_THRESHOLD)
        || goalAngle < (TurretConfig.MIN_ANGLE + TurretConfig.OUT_OF_BOUNDS_THRESHOLD);
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();
    switch (getState()) {
      case SCORE, FEED -> {
        afterTransition(getState());
        DogLog.clearFault("Turret is not homed");
      }
      case UNHOMED -> {
        DogLog.logFault("Turret is not homed", AlertType.kError);
      }
      default -> {
        DogLog.clearFault("Turret is not homed");
      }
    }
  }

  public void scoreRequest(double goalAngle) {
    this.goalAngle = goalAngle;
    setState(TurretState.SCORE);
  }

  public void feedRequest(double goalAngle) {
    this.goalAngle = goalAngle;
    setState(TurretState.FEED);
  }

  public void idleScoreRequest(double goalAngle) {
    this.goalAngle = goalAngle;
    setState(TurretState.IDLE_SCORE);
  }

  public void idleFeedRequest(double goalAngle) {
    this.goalAngle = goalAngle;
    setState(TurretState.IDLE_FEED);
  }

  public boolean atGoal() {
    return switch (getState()) {
      case UNHOMED -> false;

      // TODO: Reconsider for turret wrapping
      default ->
          MathUtil.isNear(
              goalAngle, MathHelpers.angleModulus(currentAngle), TurretConfig.TOLERANCE);
    };
  }

  public double getAngle() {
    return currentAngle;
  }

  public double getVelocityDegreesPerSecond() {
    return Units.rotationsToDegrees(motor.getVelocity().getValueAsDouble());
  }

  @Override
  public void simulationPeriodic() {
    var turretSimulation =
        SimKit.positionMechanism(
            "turret",
            (mechanism) ->
                mechanism
                    .addMotor(motor)
                    .withMinPosition(Units.degreesToRotations(TurretConfig.MIN_ANGLE))
                    .withMaxPosition(Units.degreesToRotations(TurretConfig.MAX_ANGLE)));

    if (getState() == TurretState.UNHOMED) {
      motor.setPosition(0);
      setStateFromRequest(TurretState.SCORE);
    }

    turretSimulation.update();
  }
}
