package frc.robot.turret;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team581.math.MathHelpers;
import com.team581.simkit.SimKit;
import com.team581.util.AprilTags;
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
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.vision.Vision;

public class Turret extends StateMachineSubsystem<TurretState> {
  private final TalonFX motor;
  private final CANcoder encoder;
  private double currentAngle = 0.0;
  private double goalAngle = 0.0;
  private double velocity = 0.0;
  private double voltage = 0.0;
  private double statorCurrent = 0.0;
  private double robotRotationFeedForward = 0.0;
  private double stuckAngle = 0.0;

  private final PositionVoltage positionRequest = new PositionVoltage(0.0).withEnableFOC(false);

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
    currentAngle =
        getState() == TurretState.STUCK
            ? stuckAngle
            : Units.rotationsToDegrees(motor.getPosition().getValueAsDouble());

    velocity = Units.rotationsToDegrees(motor.getVelocity().getValueAsDouble());
    voltage = motor.getMotorVoltage().getValueAsDouble();
    statorCurrent = motor.getStatorCurrent().getValueAsDouble();

    // Predict the turret's current angle to account for sensor latency
    double latencyCompensatedAngle =
        getState() == TurretState.STUCK
            ? stuckAngle
            : Units.rotationsToDegrees(
                BaseStatusSignal.getLatencyCompensatedValueAsDouble(
                    motor.getPosition(), motor.getVelocity()));

    // Add the predicted angle to the vision buffer at the current timestamp
    vision.addTurretObservation(Timer.getFPGATimestamp(), latencyCompensatedAngle, velocity);

    DogLog.log("Turret/Angle", currentAngle);
    DogLog.log("Turret/Motor/LatencyCompensatedAngle", latencyCompensatedAngle);
    DogLog.log(
        "Turret/Encoder/EncoderAngle",
        Units.rotationsToDegrees(encoder.getAbsolutePosition().getValueAsDouble()));
  }

  @Override
  protected void whileInState(TurretState currentState) {
    switch (currentState) {
      case UNHOMED -> {
        motor.disable();
      }
      case SCORE, FEED, CLIMB -> {
        motor.setControl(
            positionRequest
                .withPosition(
                    Units.degreesToRotations(
                        clamp(TurretCalculator.getOptimalAngle(goalAngle, currentAngle))))
                .withVelocity(Units.degreesToRotations(robotRotationFeedForward)));
      }
      case IDLE_SCORE, IDLE_FEED -> {
        motor.setControl(
            positionRequest
                .withPosition(
                    Units.degreesToRotations(
                        clamp(TurretCalculator.getSmartUnwrapAngle(goalAngle, currentAngle))))
                .withVelocity(Units.degreesToRotations(robotRotationFeedForward)));
      }
      case CLIMB_SCORE -> {
        motor.setControl(
            positionRequest.withPosition(
                Units.degreesToRotations(
                    clamp(TurretCalculator.getSmartUnwrapAngle(goalAngle, currentAngle)))));
      }
      case STUCK -> {
        motor.disable();
      }
      default -> {}
    }

    DogLog.log("Turret/AtGoal", atGoal());
    DogLog.log("Turret/StatorCurrent", statorCurrent);
    DogLog.log("Turret/Voltage", voltage);
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
    if (DriverStation.isDisabled() && getState() != TurretState.UNHOMED) {
      if (!MathUtil.isNear(goalAngle, MathHelpers.angleModulus(currentAngle), 10.0)) {
        DogLog.logFault("Turret is misaligned", AlertType.kWarning);
        DogLog.clearFault("Turret is misaligned");
      }
    }
  }

  public void scoreRequest(double goalAngle) {
    if (!DSOptions.USE_TURRET.getAsBoolean()) {
      stuckRequest();
      return;
    }
    this.goalAngle = goalAngle;
    setState(TurretState.SCORE);
  }

  public void climbScoreRequest(boolean isLeft) {
    if (!DSOptions.USE_TURRET.getAsBoolean()) {
      stuckRequest();
      return;
    }
    this.goalAngle = 0.0;
    setState(TurretState.CLIMB_SCORE);
  }

  public void climbRequest(Pose2d robotPose) {
    if (!DSOptions.USE_TURRET.getAsBoolean()) {
      stuckRequest();
      return;
    }
    goalAngle =
        TurretCalculator.calculateTurretAimingAngle(
            robotPose, AprilTags.getClimbTagPose().getTranslation());
    setState(TurretState.CLIMB);
  }

  public void feedRequest(double goalAngle) {
    if (!DSOptions.USE_TURRET.getAsBoolean()) {
      stuckRequest();
      return;
    }
    this.goalAngle = goalAngle;
    setState(TurretState.FEED);
  }

  public void idleScoreRequest(double goalAngle) {
    if (!DSOptions.USE_TURRET.getAsBoolean()) {
      stuckRequest();
      return;
    }
    this.goalAngle = goalAngle;
    setState(TurretState.IDLE_SCORE);
  }

  public void idleFeedRequest(double goalAngle) {
    if (!DSOptions.USE_TURRET.getAsBoolean()) {
      stuckRequest();
      return;
    }
    this.goalAngle = goalAngle;
    setState(TurretState.IDLE_FEED);
  }

  public void setRobotRotationRate(double rateDegrees) {
    robotRotationFeedForward = -rateDegrees;
  }

  public boolean atGoal(double tolerance) {
    return switch (getState()) {
      case UNHOMED -> false;
      case STUCK -> true;
      // TODO: Reconsider for turret wrapping
      default -> MathUtil.isNear(goalAngle, MathHelpers.angleModulus(currentAngle), tolerance);
    };
  }

  public boolean atGoal() {
    return atGoal(TurretConfig.TOLERANCE.get());
  }

  public void stuckRequest() {
    setStateFromRequest(TurretState.STUCK);
  }

  public void setStuckAngle(double stuckAngle) {
    this.stuckAngle = stuckAngle;
    motor.setPosition(Units.degreesToRotations(stuckAngle));
  }

  public double getAngle() {
    if (getState() == TurretState.STUCK) {
      return stuckAngle;
    }
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
}
