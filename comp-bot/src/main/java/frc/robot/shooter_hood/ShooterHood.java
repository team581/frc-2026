package frc.robot.shooter_hood;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.team581.simkit.SimKit;
import com.team581.util.state_machines.StateMachineSubsystem;
import com.team581.util.tuning.TunablePid;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import frc.robot.config.FeatureFlags;
import frc.robot.util.scheduling.SubsystemPriority;

public class ShooterHood extends StateMachineSubsystem<ShooterHoodState> {
  private static double distanceToScoringAngle(double distance) {
    return FeatureFlags.REGRESSION_MODEL.getAsBoolean()
        ? ShooterHoodConfig.SCORING_REGRESSION_MODEL.calculate(distance)
        : ShooterHoodConfig.DISTANCE_TO_SCORE.get(distance);
  }

  private static double distanceToFeedingAngle(double distance) {
    return FeatureFlags.REGRESSION_MODEL.getAsBoolean()
        ? ShooterHoodConfig.FEEDING_REGRESSION_MODEL.calculate(distance)
        : ShooterHoodConfig.DISTANCE_TO_FEED.get(distance);
  }

  private final TalonFX motor;
  private final PositionVoltage positionVoltageRequest =
      new PositionVoltage(0).withEnableFOC(false);

  private double scoreDistance = 0;
  private double feedDistance = 0;
  private double measuredAngle = 0;
  private double scoreAngle = 0;
  private double feedAngle = 0;
  private double statorCurrent = 0;

  public ShooterHood(TalonFX motor) {
    super(SubsystemPriority.SHOOTER_HOOD, ShooterHoodState.UNHOMED);

    motor.getConfigurator().apply(ShooterHoodConfig.MOTOR_CONFIG);

    this.motor = motor;

    TunablePid.register("ShooterHood", motor, ShooterHoodConfig.MOTOR_CONFIG);
  }

  public void scoreRequest(double distance) {
    this.scoreDistance = distance;
    switch (getState()) {
      case UNHOMED, HOMING -> {
        // Do nothing, we aren't homed
      }
      default -> setStateFromRequest(ShooterHoodState.SCORING);
    }
  }

  public double getAngle() {
    return measuredAngle;
  }

  public void feedRequest(double distance) {
    this.feedDistance = distance;
    switch (getState()) {
      case UNHOMED, HOMING -> {
        // Do nothing, we aren't homed
      }
      default -> setStateFromRequest(ShooterHoodState.FEEDING);
    }
  }

  public void idleRequest() {
    switch (getState()) {
      case UNHOMED, HOMING -> {
        // Do nothing, we aren't homed
      }
      default -> setStateFromRequest(ShooterHoodState.IDLE);
    }
  }

  public void homingRequest() {
    setStateFromRequest(ShooterHoodState.HOMING);
  }

  public boolean atGoal() {
    return switch (getState()) {
      case UNHOMED, HOMING -> false;
      case IDLE ->
          MathUtil.isNear(ShooterHoodConfig.IDLE_ANGLE, measuredAngle, ShooterHoodConfig.TOLERANCE);
      case SCORING -> MathUtil.isNear(scoreAngle, measuredAngle, ShooterHoodConfig.TOLERANCE);
      case FEEDING -> MathUtil.isNear(feedAngle, measuredAngle, ShooterHoodConfig.TOLERANCE);
    };
  }

  @Override
  protected void collectInputs() {
    measuredAngle = Units.rotationsToDegrees(motor.getPosition().getValueAsDouble());
    statorCurrent = motor.getStatorCurrent().getValueAsDouble();
    scoreAngle = clamp(distanceToScoringAngle(scoreDistance));
    feedAngle = clamp(distanceToFeedingAngle(feedDistance));

    DogLog.log("ShooterHood/MeasuredAngle", measuredAngle);
    DogLog.log("ShooterHood/ScoreAngle", scoreAngle);
    DogLog.log("ShooterHood/FeedingAngle", feedAngle);
    DogLog.log("ShooterHood/StatorCurrent", statorCurrent);
    switch (getState()) {
      case UNHOMED, HOMING -> {
        statorCurrent = motor.getStatorCurrent().getValueAsDouble();
      }
      default -> {}
    }
  }

  @Override
  protected ShooterHoodState getNextState(ShooterHoodState currentState) {
    return switch (currentState) {
      case HOMING -> {
        if (statorCurrent >= ShooterHoodConfig.HOMING_CURRENT_THRESHOLD) {
          motor.setPosition(ShooterHoodConfig.HOMING_END_POSITION);
          yield ShooterHoodState.IDLE;
        }
        yield currentState;
      }
      default -> currentState;
    };
  }

  private static double clamp(double angleDegrees) {
    return MathUtil.clamp(angleDegrees, ShooterHoodConfig.MIN_ANGLE, ShooterHoodConfig.MAX_ANGLE);
  }

  @Override
  protected void afterTransition(ShooterHoodState newState) {
    switch (newState) {
      case HOMING -> motor.setVoltage(ShooterHoodConfig.HOMING_VOLTAGE);

      case UNHOMED -> motor.disable();

      case IDLE -> {
        motor.setControl(
            positionVoltageRequest.withPosition(
                Units.degreesToRotations(clamp(ShooterHoodConfig.IDLE_ANGLE))));
        DogLog.log("ShooterHood/CurrentSetpoint", ShooterHoodConfig.IDLE_ANGLE);
      }

      default -> {}
    }
  }

  @Override
  protected void whileInState(ShooterHoodState state) {
    switch (state) {
      case SCORING -> {
        motor.setControl(
            positionVoltageRequest.withPosition(Units.degreesToRotations(clamp(scoreAngle))));
        DogLog.log("ShooterHood/CurrentSetpoint", scoreAngle);
      }

      case FEEDING -> {
        motor.setControl(
            positionVoltageRequest.withPosition(Units.degreesToRotations(clamp(feedAngle))));
        DogLog.log("ShooterHood/CurrentSetpoint", feedAngle);
      }

      default -> {}
    }

    DogLog.log("ShooterHood/AtGoal", atGoal());
  }

  @Override
  public void simulationPeriodic() {
    var shooterHoodSimulation =
        SimKit.positionMechanism(
            "ShooterHood",
            (mechanism) ->
                mechanism
                    .addMotor(motor, ChassisReference.CounterClockwise_Positive)
                    .withMaxPosition(Units.degreesToRotations(ShooterHoodConfig.MAX_ANGLE))
                    .withMinPosition(Units.degreesToRotations(ShooterHoodConfig.MIN_ANGLE)));

    if (getState() == ShooterHoodState.HOMING) {
      motor.setPosition(0);
      setStateFromRequest(ShooterHoodState.IDLE);
    }

    shooterHoodSimulation.update();
  }
}
