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
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
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
        Units.rotationsToDegrees(encoder.getPosition().getValueAsDouble()));
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
  }

  public void scoreRequest(double goalAngle) {
    this.goalAngle = goalAngle;
    setState(TurretState.SCORE);
  }

  public void climbScoreRequest(boolean isLeft) {
    this.goalAngle = 0.0;
    setState(TurretState.CLIMB_SCORE);
  }

  public void climbRequest(Pose2d robotPose) {
    goalAngle =
        TurretCalculator.calculateTurretAimingAngle(
            robotPose, AprilTags.getClimbTagPose().getTranslation());
    setState(TurretState.CLIMB);
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

  public void setRobotRotationRate(double rateDegrees) {
    robotRotationFeedForward = -rateDegrees;
  }

  public boolean atGoal() {
    return switch (getState()) {
      case UNHOMED -> false;

      // TODO: Reconsider for turret wrapping
      default ->
          MathUtil.isNear(
              goalAngle, MathHelpers.angleModulus(currentAngle), TurretConfig.TOLERANCE.get());
    };
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
}
