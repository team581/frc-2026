package frc.robot.shooter;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.team581.math.MathHelpers;
import com.team581.mechanisms.PowerManaged;
import com.team581.simkit.SimKit;
import com.team581.util.state_machines.StateMachineSubsystem;
import com.team581.util.tuning.TunablePid;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import frc.robot.config.DSOptions;
import frc.robot.config.FeatureFlags;
import frc.robot.util.scheduling.SubsystemPriority;

public class Shooter extends StateMachineSubsystem<ShooterState> implements PowerManaged {
  private static double distanceToScoringRpm(double distance) {
    return FeatureFlags.REGRESSION_MODEL.getAsBoolean()
        ? ShooterConfig.SCORING_REGRESSION_MODEL.calculate(distance)
        : ShooterConfig.DISTANCE_TO_SCORE_RPM.get(distance);
  }

  private static double distanceToFeedingRpm(double distance) {
    return FeatureFlags.REGRESSION_MODEL.getAsBoolean()
        ? ShooterConfig.FEEDING_REGRESSION_MODEL.calculate(distance)
        : ShooterConfig.DISTANCE_TO_FEEDING_RPM.get(distance);
  }

  private final TalonFX leftMotor;
  private final TalonFX rightMotor;

  private final VelocityVoltage voltageRequest =
      new VelocityVoltage(0).withLimitReverseMotion(true).withEnableFOC(false);

  private double scoreDistance = 0;
  private double climbScoreRpm = 0;
  private double feedDistance = 0;

  private double shootingRpm = 0;
  private double feedingRpm = 0;
  private double leftMotorRpm = 0;
  private double rightMotorRpm = 0;

  private double leftStatorCurrent = 0.0;
  private double rightStatorCurrent = 0.0;

  private boolean atGoal = false;
  private boolean atGoalDebounced = false;

  // Debounce for delay between shots at 15 bps
  private final Debouncer atGoalDebouncer = new Debouncer(1.0 / 15.0, DebounceType.kFalling);

  public Shooter(TalonFX leftMotor, TalonFX rightMotor) {
    super(SubsystemPriority.SHOOTER, ShooterState.IDLE);

    leftMotor.getConfigurator().apply(ShooterConfig.LEFT_MOTOR_CONFIGS);
    rightMotor.getConfigurator().apply(ShooterConfig.RIGHT_MOTOR_CONFIG);

    TunablePid.register("Shooter/LeftShooter", leftMotor, ShooterConfig.LEFT_MOTOR_CONFIGS);
    TunablePid.register("Shooter/RightShooter", rightMotor, ShooterConfig.RIGHT_MOTOR_CONFIG);

    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
  }

  public void scoreRequest(double distance) {
    this.scoreDistance = distance;
    setStateFromRequest(ShooterState.SCORE);
  }

  public void climbScoreRequest(boolean isLeft) {
    climbScoreRpm = 0.0;
    setStateFromRequest(ShooterState.CLIMB_SCORE);
  }

  public void feedRequest(double distance) {
    this.feedDistance = distance;
    setStateFromRequest(ShooterState.FEEDING);
  }

  public void idleRequest() {
    setStateFromRequest(ShooterState.IDLE);
  }

  @Override
  protected void whileInState(ShooterState state) {
    DogLog.log("Shooter/Left/RPM", leftMotorRpm);
    DogLog.log("Shooter/Right/RPM", rightMotorRpm);
    DogLog.log("Shooter/GoalShootingRPM", shootingRpm);
    DogLog.log("Shooter/GoalFeedingRPM", feedingRpm);
    DogLog.log("Shooter/AtGoal", atGoal());
    DogLog.log("Shooter/Right/Voltage", rightMotor.getMotorVoltage().getValueAsDouble());
    DogLog.log("Shooter/Left/Voltage", leftMotor.getMotorVoltage().getValueAsDouble());

    DogLog.log("Shooter/Left/StatorCurrent", leftStatorCurrent);
    DogLog.log("Shooter/Right/StatorCurrent", rightStatorCurrent);
    DogLog.log("Shooter/Left/SupplyCurrent", leftMotor.getSupplyCurrent().getValueAsDouble());
    DogLog.log("Shooter/Right/SupplyCurrent", rightMotor.getSupplyCurrent().getValueAsDouble());

    switch (state) {
      case IDLE -> {
        var setpoint = ShooterConfig.IDLE_RPM / 60.0;
        leftMotor.setControl(voltageRequest.withVelocity(setpoint));
        rightMotor.setControl(voltageRequest.withVelocity(setpoint));

        DogLog.log("Shooter/RpmSetpoint", shootingRpm);
      }
      case SCORE -> {
        var setpoint = shootingRpm / 60.0;
        leftMotor.setControl(voltageRequest.withVelocity(setpoint));
        rightMotor.setControl(voltageRequest.withVelocity(setpoint));

        DogLog.log("Shooter/RpmSetpoint", shootingRpm);
      }
      case CLIMB_SCORE -> {
        var setpoint = climbScoreRpm / 60.0;
        leftMotor.setControl(voltageRequest.withVelocity(setpoint));
        rightMotor.setControl(voltageRequest.withVelocity(setpoint));

        DogLog.log("Shooter/RpmSetpoint", climbScoreRpm);
      }
      case FEEDING -> {
        var setpoint = feedingRpm / 60.0;
        leftMotor.setControl(voltageRequest.withVelocity(setpoint));
        rightMotor.setControl(voltageRequest.withVelocity(setpoint));

        DogLog.log("Shooter/RpmSetpoint", feedingRpm);
      }
      case SELF_TEST_STOP_MOTORS -> {
        leftMotor.stopMotor();
        rightMotor.stopMotor();
      }
      case SELF_TEST_LEFT_MOTOR -> {
        leftMotor.setVoltage(ShooterConfig.TEST_VOLTAGE);
        rightMotor.stopMotor();
      }
      case SELF_TEST_RIGHT_MOTOR -> {
        rightMotor.setVoltage(ShooterConfig.TEST_VOLTAGE);
        leftMotor.stopMotor();
      }
    }
  }

  @Override
  protected void collectInputs() {
    shootingRpm = Math.min(ShooterConfig.MAX_SAFE_RPM, distanceToScoringRpm(scoreDistance));
    feedingRpm = Math.min(ShooterConfig.MAX_SAFE_RPM, distanceToFeedingRpm(feedDistance));

    if (DSOptions.PIT_FUNCTIONALITY.getAsBoolean()) {
      shootingRpm = ShooterConfig.PIT_FUNCTIONALITY_RPM;
      feedingRpm = ShooterConfig.PIT_FUNCTIONALITY_RPM;
    }

    leftStatorCurrent = leftMotor.getStatorCurrent().getValueAsDouble();
    rightStatorCurrent = rightMotor.getStatorCurrent().getValueAsDouble();

    leftMotorRpm = leftMotor.getVelocity().getValueAsDouble() * 60.0;
    rightMotorRpm = rightMotor.getVelocity().getValueAsDouble() * 60.0;

    atGoal = calculateAtGoal();
    atGoalDebounced = atGoalDebouncer.calculate(atGoal);

    if (getState() == ShooterState.SELF_TEST_LEFT_MOTOR) {
      DogLog.log(
          "Shooter/SelfTest/LeftMotor/VelocityGood",
          MathUtil.isNear(
              ShooterConfig.SELF_TEST_LEFT_MOTOR_EXPECTED_RPM,
              leftMotorRpm,
              ShooterConfig.SELF_TEST_RIGHT_MOTOR_RPM_TOLERANCE));
      DogLog.log(
          "Shooter/SelfTest/LeftMotor/CurrentGood",
          MathUtil.isNear(
              ShooterConfig.SELF_TEST_LEFT_MOTOR_EXPECTED_CURRENT,
              leftStatorCurrent,
              ShooterConfig.SELF_TEST_RIGHT_MOTOR_CURRENT_TOLERANCE));
    }

    if (getState() == ShooterState.SELF_TEST_RIGHT_MOTOR) {
      DogLog.log(
          "Shooter/SelfTest/RightMotor/VelocityGood",
          MathUtil.isNear(
              ShooterConfig.SELF_TEST_RIGHT_MOTOR_EXPECTED_RPM,
              rightMotorRpm,
              ShooterConfig.SELF_TEST_LEFT_MOTOR_RPM_TOLERANCE));
      DogLog.log(
          "Shooter/SelfTest/RightMotor/CurrentGood",
          MathUtil.isNear(
              ShooterConfig.SELF_TEST_RIGHT_MOTOR_EXPECTED_CURRENT,
              rightStatorCurrent,
              ShooterConfig.SELF_TEST_LEFT_MOTOR_CURRENT_TOLERANCE));
    }
  }

  public boolean atGoal() {
    return atGoal;
  }

  public boolean atGoalDebounced() {
    return atGoalDebounced;
  }

  private boolean calculateAtGoal() {
    return switch (getState()) {
      case IDLE -> false;
      case SCORE ->
          MathUtil.isNear(leftMotorRpm, shootingRpm, ShooterConfig.RPM_TOLERANCE)
              && MathUtil.isNear(rightMotorRpm, shootingRpm, ShooterConfig.RPM_TOLERANCE);
      case CLIMB_SCORE ->
          MathUtil.isNear(leftMotorRpm, climbScoreRpm, ShooterConfig.RPM_TOLERANCE)
              && MathUtil.isNear(rightMotorRpm, climbScoreRpm, ShooterConfig.RPM_TOLERANCE);

      case FEEDING ->
          MathUtil.isNear(leftMotorRpm, feedingRpm, ShooterConfig.RPM_TOLERANCE_FEEDING)
              && MathUtil.isNear(rightMotorRpm, feedingRpm, ShooterConfig.RPM_TOLERANCE_FEEDING);

      default -> true;
    };
  }

  @Override
  public void simulationPeriodic() {
    var shooterSimulation =
        SimKit.velocityMechanism(
            "shooter",
            (mechanism) ->
                mechanism
                    .addMotor(leftMotor, ChassisReference.CounterClockwise_Positive)
                    .addMotor(rightMotor, ChassisReference.Clockwise_Positive));

    shooterSimulation.update();
  }

  public boolean currentDetectsGp() {
    return ShooterConfig.GP_DETECT_CURRENT_THRESHOLD
        <= MathHelpers.average(rightStatorCurrent, leftStatorCurrent);
  }

  public double getScoreTimeOfFlight(double distance) {
    this.scoreDistance = distance;
    return FeatureFlags.TOF_REGRESSION_MODEL.getAsBoolean()
        ? ShooterConfig.SCORING_TOF_REGRESSION_MODEL.calculate(scoreDistance)
        : ShooterConfig.DISTANCE_TO_SCORE_TOF.get(scoreDistance);
  }

  public double getFeedTimeOfFlight(double distance) {
    this.feedDistance = distance;
    return FeatureFlags.TOF_REGRESSION_MODEL.getAsBoolean()
        ? ShooterConfig.FEEDING_TOF_REGRESSION_MODEL.calculate(feedDistance)
        : ShooterConfig.DISTANCE_TO_FEED_TOF.get(feedDistance);
  }

  @Override
  public void applyCurrentLimits(double supplyCurrentLimit) {
    leftMotor
        .getConfigurator()
        .apply(
            ShooterConfig.LEFT_MOTOR_CONFIGS.CurrentLimits.withSupplyCurrentLimit(
                supplyCurrentLimit));
    rightMotor
        .getConfigurator()
        .apply(
            ShooterConfig.RIGHT_MOTOR_CONFIG.CurrentLimits.withSupplyCurrentLimit(
                supplyCurrentLimit));
  }
}
