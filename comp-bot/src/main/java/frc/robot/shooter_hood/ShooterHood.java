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
  private double currentAngle = 0;
  private double statorCurrent = 0;
  private double climbScoreAngle = 0;

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
    return currentAngle;
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

  public void climbScoreRequest(boolean isLeft) {
    climbScoreAngle = 0.0;
    setStateFromRequest(ShooterHoodState.CLIMB_SCORING);
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
      default -> MathUtil.isNear(getGoalAngle(), currentAngle, ShooterHoodConfig.TOLERANCE);
    };
  }

  @Override
  protected void collectInputs() {
    currentAngle = Units.rotationsToDegrees(motor.getPosition().getValueAsDouble());
    statorCurrent = motor.getStatorCurrent().getValueAsDouble();
  }

  private double getGoalAngle() {
    return switch (getState()) {
      case UNHOMED, HOMING -> -1;
      case IDLE -> ShooterHoodConfig.IDLE_ANGLE;
      case SCORING -> distanceToScoringAngle(scoreDistance);
      case FEEDING -> distanceToFeedingAngle(feedDistance);
      case CLIMB_SCORING -> climbScoreAngle;
    };
  }

  @Override
  protected ShooterHoodState getNextState(ShooterHoodState currentState) {
    return switch (currentState) {
      case HOMING -> {
        if (statorCurrent >= ShooterHoodConfig.HOMING_CURRENT_THRESHOLD) {
          motor.setPosition(Units.degreesToRotations(ShooterHoodConfig.HOMING_END_POSITION));
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

      default -> {
        motor.setControl(
            positionVoltageRequest.withPosition(Units.degreesToRotations(clamp(getGoalAngle()))));
      }
    }
  }

  @Override
  protected void whileInState(ShooterHoodState state) {
    var goalAngle = getGoalAngle();

    switch (state) {
      case SCORING, FEEDING -> {
        motor.setControl(
            positionVoltageRequest.withPosition(Units.degreesToRotations(clamp(goalAngle))));
      }
      case CLIMB_SCORING -> {
        motor.setControl(
            positionVoltageRequest.withPosition(Units.degreesToRotations(clamp(climbScoreAngle))));
        DogLog.log("ShooterHood/CurrentSetpoint", climbScoreAngle);
      }
      // Do nothing in the other states, they have static setpoints
      default -> {}
    }

    DogLog.log("ShooterHood/AtGoal", atGoal());
    DogLog.log("ShooterHood/Angle", currentAngle);
    DogLog.log("ShooterHood/GoalAngle", goalAngle);
    DogLog.log("ShooterHood/Motor/StatorCurrent", statorCurrent);
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
                    .withMinPosition(
                        Units.degreesToRotations(ShooterHoodConfig.HOMING_END_POSITION)));

    if (getState() == ShooterHoodState.HOMING) {
      motor.setPosition(Units.degreesToRotations(ShooterHoodConfig.HOMING_END_POSITION));
      setStateFromRequest(ShooterHoodState.IDLE);
    }

    shooterHoodSimulation.update();
  }
}
