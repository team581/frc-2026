package frc.robot.turret;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team581.mechanisms.PowerManaged;
import com.team581.simkit.SimKit;
import com.team581.util.state_machines.StateMachineSubsystem;
import com.team581.util.tuning.TunablePid;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.config.DSOptions;
import frc.robot.util.AimParameterUtil.AimingParameters;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.vision.Vision;

public class Turret extends StateMachineSubsystem<TurretState> implements PowerManaged {
  private final TalonFX motor;
  private final CANcoder encoder;
  private double currentAngle = 0.0;
  private double goalAngle = 0.0;
  private double setpoint = 0.0;
  private double velocity = 0.0;
  private double voltage = 0.0;
  private double statorCurrent = 0.0;
  private double feedForward = 0.0;
  private double stuckAngle = 0.0;

  private final PositionVoltage positionRequest = new PositionVoltage(0.0).withEnableFOC(false);
  private final NeutralOut neutralRequest = new NeutralOut();

  private final Vision vision;

  public Turret(TalonFX motor, CANcoder encoder, Vision vision) {
    super(SubsystemPriority.TURRET, TurretState.UNHOMED);
    this.vision = vision;

    motor.getConfigurator().apply(TurretConfig.MOTOR_CONFIG);
    encoder.getConfigurator().apply(TurretConfig.ENCODER_CONFIG);

    TunablePid.register("Turret", motor, TurretConfig.MOTOR_CONFIG);

    this.motor = motor;
    this.encoder = encoder;
  }

  @Override
  protected TurretState getNextState(TurretState currentState) {
    switch (currentState) {
      case UNHOMED -> {
        if (motor.isAlive() && motor.isConnected() && encoder.isConnected() && RobotBase.isReal()) {
          double motorPosition = motor.getRotorPosition().getValueAsDouble();
          double encoderPosition = encoder.getAbsolutePosition().getValueAsDouble();
          var turretPos =
              TurretCalculator.calculateHomedPositionFromMotorAndEncoder(
                  motorPosition, encoderPosition);
          motor.setPosition(turretPos);
          return TurretState.SCORE;
        } else {
          return currentState;
        }
      }
      default -> {
        return currentState;
      }
    }
  }

  @Override
  protected void collectInputs() {
    currentAngle = Units.rotationsToDegrees(motor.getPosition().getValueAsDouble());

    velocity = Units.rotationsToDegrees(motor.getVelocity().getValueAsDouble());
    voltage = motor.getMotorVoltage().getValueAsDouble();
    statorCurrent = motor.getStatorCurrent().getValueAsDouble();

    // Predict the turret's current angle to account for sensor latency
    double latencyCompensatedAngle =
        Units.rotationsToDegrees(
            BaseStatusSignal.getLatencyCompensatedValueAsDouble(
                motor.getPosition(), motor.getVelocity()));

    // Add the predicted angle to the vision buffer at the current timestamp
    vision.addTurretObservation(Timer.getFPGATimestamp(), latencyCompensatedAngle, velocity);

    DogLog.log("Turret/Angle", currentAngle);
    DogLog.log("Turret/Motor/LatencyCompensatedAngle", latencyCompensatedAngle);
    DogLog.log(
        "Turret/Encoder/EncoderAngle",
        Units.rotationsToDegrees(encoder.getAbsolutePosition().getValueAsDouble()));

    setpoint =
        switch (getState()) {
          case UNHOMED -> 0.0;
          case SCORE, FEED, CLIMB, CLIMB_SCORE, STUCK ->
              clamp(TurretCalculator.getOptimalAngle(goalAngle, currentAngle));
          case IDLE_SCORE, IDLE_FEED ->
              clamp(TurretCalculator.getSmartUnwrapAngle(goalAngle, currentAngle));
        };
  }

  @Override
  protected void whileInState(TurretState currentState) {
    switch (currentState) {
      case UNHOMED -> motor.setControl(neutralRequest);
      case STUCK -> {
        motor.setControl(
            positionRequest.withPosition(
                Units.degreesToRotations(clamp(TurretConfig.FAUX_DUMPER_ANGLE))));
      }
      case SCORE, FEED, CLIMB ->
          motor.setControl(
              positionRequest
                  .withPosition(Units.degreesToRotations(clamp(setpoint)))
                  .withVelocity(Units.radiansToRotations(getFeedForward())));
      case IDLE_SCORE, IDLE_FEED ->
          motor.setControl(
              positionRequest
                  .withPosition(Units.degreesToRotations(clamp(setpoint)))
                  .withVelocity(Units.radiansToRotations(getFeedForward())));
      case CLIMB_SCORE ->
          motor.setControl(
              positionRequest
                  .withPosition(Units.degreesToRotations(clamp(setpoint)))
                  .withVelocity(Units.radiansToRotations(getFeedForward())));
      default -> {}
    }

    DogLog.log("Turret/StatorCurrent", statorCurrent);
    DogLog.log("Turret/Voltage", voltage);
    DogLog.log("Turret/Setpoint", setpoint);
  }

  private double getFeedForward() {
    if (MathUtil.isNear(TurretConfig.MAX_ANGLE, currentAngle, 3)
        || MathUtil.isNear(TurretConfig.MIN_ANGLE, currentAngle, 3)) {
      return 0.0;
    }
    return feedForward;
  }

  public void setState(TurretState newState) {
    switch (getState()) {
      case UNHOMED -> {}
      default -> setStateFromRequest(newState);
    }
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();
    switch (getState()) {
      case SCORE, FEED -> afterTransition(getState());
      default -> {}
    }

    if (getState() == TurretState.UNHOMED) {
      DogLog.logFault("Turret is not homed", AlertType.kError);
    } else {
      DogLog.clearFault("Turret is not homed");
    }

    if (getState() != TurretState.UNHOMED
        && DriverStation.isDisabled()
        && DriverStation.isAutonomous()
        && !MathUtil.isNear(setpoint, currentAngle, 10.0)) {
      DogLog.logFault("Turret is misaligned", AlertType.kWarning);
    } else {
      DogLog.clearFault("Turret is misaligned");
    }
  }

  public void scoreRequest(double goalAngle, double feedForward) {
    this.feedForward = feedForward;
    if (!DSOptions.USE_TURRET.getAsBoolean()) {
      stuckRequest();
      return;
    }
    this.goalAngle = goalAngle;
    setState(TurretState.SCORE);
  }

  public void climbScoreRequest(boolean isLeft, double feedForward) {
    this.feedForward = feedForward;
    if (!DSOptions.USE_TURRET.getAsBoolean()) {
      stuckRequest();
      return;
    }
    this.goalAngle = 0.0;
    setState(TurretState.CLIMB_SCORE);
  }

  public void climbRequest(Pose2d robotPose, double feedForward) {
    this.feedForward = feedForward;
    if (!DSOptions.USE_TURRET.getAsBoolean()) {
      stuckRequest();
      return;
    }
    goalAngle = 0.0;
    setState(TurretState.CLIMB);
  }

  public void feedRequest(double goalAngle, double feedForward) {
    this.feedForward = feedForward;
    if (!DSOptions.USE_TURRET.getAsBoolean()) {
      stuckRequest();
      return;
    }
    this.goalAngle = goalAngle;
    setState(TurretState.FEED);
  }

  public void idleScoreRequest(double goalAngle, double feedForward) {
    this.feedForward = feedForward;
    if (!DSOptions.USE_TURRET.getAsBoolean()) {
      stuckRequest();
      return;
    }
    this.goalAngle = goalAngle;
    setState(TurretState.IDLE_SCORE);
  }

  public void idleFeedRequest(double goalAngle, double feedForward) {
    this.feedForward = feedForward;
    if (!DSOptions.USE_TURRET.getAsBoolean()) {
      stuckRequest();
      return;
    }
    this.goalAngle = goalAngle;
    setState(TurretState.IDLE_FEED);
  }

  public boolean atGoal(double tolerance) {
    return switch (getState()) {
      case UNHOMED -> false;
      case STUCK -> true;
      default -> MathUtil.isNear(setpoint, currentAngle, tolerance);
    };
  }

  public boolean atGoal(AimingParameters aimingParameters) {
    return atGoal(aimingParameters.turretTolerance(), aimingParameters.upcomingTurretAngle());
  }

  public boolean atGoal(double tolerance, double upcomingAngle) {
    return switch (getState()) {
      case UNHOMED -> false;
      case STUCK -> true;
      default -> {
        var potentialSetpoint = TurretCalculator.getOptimalAngle(upcomingAngle, currentAngle);
        if (!MathUtil.isNear(potentialSetpoint, setpoint, 90)) {
          yield false;
        }
        yield MathUtil.isNear(setpoint, currentAngle, tolerance, -180, 180);
      }
    };
  }

  public void stuckRequest() {
    if (getState() != TurretState.UNHOMED) {
      setStateFromRequest(TurretState.STUCK);
    }
  }

  public double getAngle() {
    return currentAngle;
  }

  public double getVelocity() {
    return velocity;
  }

  private static double clamp(double wantedAngle) {
    return MathUtil.clamp(wantedAngle, TurretConfig.MIN_ANGLE, TurretConfig.MAX_ANGLE);
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

  @Override
  public void applyCurrentLimits(double supplyCurrentLimit) {
    motor
        .getConfigurator()
        .apply(TurretConfig.MOTOR_CONFIG.CurrentLimits.withSupplyCurrentLimit(supplyCurrentLimit));
  }
}
