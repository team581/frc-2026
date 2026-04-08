package frc.robot.shooter;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.sim.ChassisReference;
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

  // Top left motor is the leader
  private final TalonFX topLeftMotor;
  private final TalonFX topRightMotor;
  public final TalonFX bottomLeftMotor;
  public final TalonFX bottomRightMotor;

  private final Follower topLeftFollower;
  private final Follower bottomLeftFollower;
  private final Follower bottomRightFollower;

  private final VelocityTorqueCurrentFOC velocityRequest = new VelocityTorqueCurrentFOC(0);

  private double scoreDistance = 0;
  private double feedDistance = 0;

  private double shootingRpm = 0;
  private double feedingRpm = 0;
  private double topLeftMotorRpm = 0;
  private double topRightMotorRpm = 0;
  private double bottomLeftMotorRpm = 0;
  private double bottomRightMotorRpm = 0;

  private double topLeftStatorCurrent = 0.0;
  private double topRightStatorCurrent = 0.0;
  private double bottomLeftStatorCurrent = 0.0;
  private double bottomRightStatorCurrent = 0.0;

  private boolean atGoal = false;
  private boolean atGoalDebounced = false;

  private boolean turboMode = false;

  // Debounce for delay between shots at 15 bps
  private final Debouncer atGoalDebouncer = new Debouncer(1.0 / 15.0, DebounceType.kFalling);

  public Shooter(
      TalonFX topLeftMotor,
      TalonFX topRightMotor,
      TalonFX bottomLeftMotor,
      TalonFX bottomRightMotor) {
    super(SubsystemPriority.SHOOTER, ShooterState.IDLE);

    topLeftMotor.getConfigurator().apply(ShooterConfig.TOP_LEFT_MOTOR_CONFIGS);
    topRightMotor.getConfigurator().apply(ShooterConfig.TOP_RIGHT_MOTOR_CONFIG);
    bottomLeftMotor.getConfigurator().apply(ShooterConfig.BOTTOM_LEFT_MOTOR_CONFIG);
    bottomRightMotor.getConfigurator().apply(ShooterConfig.BOTTOM_RIGHT_MOTOR_CONFIG);

    TunablePid.register("Shooter/TopLeft", topLeftMotor, ShooterConfig.TOP_LEFT_MOTOR_CONFIGS);
    TunablePid.register("Shooter/TopRight", topRightMotor, ShooterConfig.TOP_RIGHT_MOTOR_CONFIG);
    TunablePid.register(
        "Shooter/BottomLeft", bottomLeftMotor, ShooterConfig.BOTTOM_LEFT_MOTOR_CONFIG);
    TunablePid.register(
        "Shooter/BottomRight", bottomRightMotor, ShooterConfig.BOTTOM_RIGHT_MOTOR_CONFIG);

    this.topLeftMotor = topLeftMotor;
    this.topRightMotor = topRightMotor;
    this.bottomLeftMotor = bottomLeftMotor;
    this.bottomRightMotor = bottomRightMotor;

    this.topLeftFollower = new Follower(topRightMotor.getDeviceID(), MotorAlignmentValue.Opposed);
    this.bottomLeftFollower =
        new Follower(topRightMotor.getDeviceID(), MotorAlignmentValue.Opposed);
    this.bottomRightFollower =
        new Follower(topRightMotor.getDeviceID(), MotorAlignmentValue.Aligned);

    topLeftMotor.setControl(topLeftFollower);
    bottomLeftMotor.setControl(bottomLeftFollower);
    bottomRightMotor.setControl(bottomRightFollower);
  }

  public void prepareScoreRequest(double distance) {
    this.scoreDistance = distance;
    if (getState() != ShooterState.SCORE) {
      setStateFromRequest(ShooterState.PREPARE_SCORE);
    }
  }

  public void scoreRequest(double distance) {
    this.scoreDistance = distance;
    setStateFromRequest(ShooterState.SCORE);
  }

  public void prepareFeedRequest(double distance) {
    this.feedDistance = distance;
    setStateFromRequest(ShooterState.PREPARE_FEED);
  }

  public void feedRequest(double distance) {
    this.feedDistance = distance;
    setStateFromRequest(ShooterState.FEED);
  }

  public void idleRequest() {
    setStateFromRequest(ShooterState.IDLE);
  }

  public void setTurboMode(boolean useTurboMode) {
    turboMode = useTurboMode;
  }

  @Override
  protected void whileInState(ShooterState state) {
    DogLog.log("Shooter/TopLeft/RPM", topLeftMotorRpm);
    DogLog.log("Shooter/TopRight/RPM", topRightMotorRpm);
    DogLog.log("Shooter/BottomLeft/RPM", bottomLeftMotorRpm);
    DogLog.log("Shooter/BottomRight/RPM", bottomRightMotorRpm);
    DogLog.log("Shooter/GoalShootingRPM", shootingRpm);
    DogLog.log("Shooter/GoalFeedingRPM", feedingRpm);
    DogLog.log("Shooter/AtGoal", atGoal());
    DogLog.log("Shooter/TopRight/Voltage", topRightMotor.getMotorVoltage().getValueAsDouble());
    DogLog.log(
        "Shooter/TopRight/SupplyVoltage", topRightMotor.getSupplyVoltage().getValueAsDouble());

    DogLog.log("Shooter/TopLeft/Voltage", topLeftMotor.getMotorVoltage().getValueAsDouble());
    DogLog.log(
        "Shooter/BottomRight/Voltage", bottomRightMotor.getMotorVoltage().getValueAsDouble());
    DogLog.log("Shooter/BottomLeft/Voltage", bottomLeftMotor.getMotorVoltage().getValueAsDouble());

    DogLog.log("Shooter/TopLeft/StatorCurrent", topLeftStatorCurrent);
    DogLog.log("Shooter/TopRight/StatorCurrent", topRightStatorCurrent);
    DogLog.log("Shooter/BottomLeft/StatorCurrent", bottomLeftStatorCurrent);
    DogLog.log("Shooter/BottomRight/StatorCurrent", bottomRightStatorCurrent);
    DogLog.log("Shooter/TopLeft/SupplyCurrent", topLeftMotor.getSupplyCurrent().getValueAsDouble());
    DogLog.log(
        "Shooter/TopRight/SupplyCurrent", topRightMotor.getSupplyCurrent().getValueAsDouble());
    DogLog.log(
        "Shooter/BottomLeft/SupplyCurrent", bottomLeftMotor.getSupplyCurrent().getValueAsDouble());
    DogLog.log(
        "Shooter/BottomRight/SupplyCurrent",
        bottomRightMotor.getSupplyCurrent().getValueAsDouble());

    DogLog.log(
        "Shooter/TopRight/TorqueCurrent", topRightMotor.getTorqueCurrent().getValueAsDouble());

    switch (state) {
      case IDLE -> {
        var setpoint = ShooterConfig.IDLE_RPM / 60.0;
        topRightMotor.setControl(velocityRequest.withVelocity(setpoint).withFeedForward(0.0));
        DogLog.log("Shooter/RpmSetpoint", ShooterConfig.IDLE_RPM);
      }
      case PREPARE_SCORE -> {
        var setpoint = shootingRpm / 60.0;
        topRightMotor.setControl(velocityRequest.withVelocity(setpoint).withFeedForward(0.0));
        DogLog.log("Shooter/RpmSetpoint", shootingRpm);
      }
      case SCORE -> {
        var setpoint = shootingRpm / 60.0;
        topRightMotor.setControl(
            velocityRequest
                .withVelocity(setpoint)
                .withFeedForward(
                    turboMode
                        ? ShooterConfig.TURBO_MODE_FF_CURRENT.get()
                        : ShooterConfig.ACTIVE_SHOT_FF_CURRENT.get()));
        DogLog.log("Shooter/RpmSetpoint", shootingRpm);
      }
      case PREPARE_FEED -> {
        var setpoint = feedingRpm / 60.0;
        topRightMotor.setControl(velocityRequest.withVelocity(setpoint).withFeedForward(0.0));
        DogLog.log("Shooter/RpmSetpoint", feedingRpm);
      }
      case FEED -> {
        var setpoint = feedingRpm / 60.0;
        topRightMotor.setControl(
            velocityRequest
                .withVelocity(setpoint)
                .withFeedForward(
                    turboMode
                        ? ShooterConfig.TURBO_MODE_FF_CURRENT.get()
                        : ShooterConfig.ACTIVE_SHOT_FF_CURRENT.get()));
        DogLog.log("Shooter/RpmSetpoint", feedingRpm);
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

    topLeftStatorCurrent = topLeftMotor.getStatorCurrent().getValueAsDouble();
    topRightStatorCurrent = topRightMotor.getStatorCurrent().getValueAsDouble();
    bottomLeftStatorCurrent = bottomLeftMotor.getStatorCurrent().getValueAsDouble();
    bottomRightStatorCurrent = bottomRightMotor.getStatorCurrent().getValueAsDouble();

    topLeftMotorRpm = topLeftMotor.getVelocity().getValueAsDouble() * 60.0;

    topRightMotorRpm = topRightMotor.getVelocity().getValueAsDouble() * 60.0;

    bottomLeftMotorRpm = bottomLeftMotor.getVelocity().getValueAsDouble() * 60.0;

    bottomRightMotorRpm = bottomRightMotor.getVelocity().getValueAsDouble() * 60.0;

    atGoal = calculateAtGoal();
    atGoalDebounced = atGoalDebouncer.calculate(atGoal);
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
      case PREPARE_SCORE ->
          MathUtil.isNear(topLeftMotorRpm, shootingRpm, ShooterConfig.RPM_TOLERANCE)
              && MathUtil.isNear(topRightMotorRpm, shootingRpm, ShooterConfig.RPM_TOLERANCE)
              && MathUtil.isNear(bottomLeftMotorRpm, shootingRpm, ShooterConfig.RPM_TOLERANCE)
              && MathUtil.isNear(bottomRightMotorRpm, shootingRpm, ShooterConfig.RPM_TOLERANCE);
      case SCORE ->
          MathUtil.isNear(
                  topLeftMotorRpm, shootingRpm, ShooterConfig.RPM_TOLERANCE_ACTIVELY_SHOOTING)
              && MathUtil.isNear(
                  topRightMotorRpm, shootingRpm, ShooterConfig.RPM_TOLERANCE_ACTIVELY_SHOOTING)
              && MathUtil.isNear(
                  bottomLeftMotorRpm, shootingRpm, ShooterConfig.RPM_TOLERANCE_ACTIVELY_SHOOTING)
              && MathUtil.isNear(
                  bottomRightMotorRpm, shootingRpm, ShooterConfig.RPM_TOLERANCE_ACTIVELY_SHOOTING);
      case PREPARE_FEED, FEED ->
          MathUtil.isNear(topLeftMotorRpm, feedingRpm, ShooterConfig.RPM_TOLERANCE_FEEDING)
              && MathUtil.isNear(topRightMotorRpm, feedingRpm, ShooterConfig.RPM_TOLERANCE_FEEDING)
              && MathUtil.isNear(
                  bottomLeftMotorRpm, feedingRpm, ShooterConfig.RPM_TOLERANCE_FEEDING)
              && MathUtil.isNear(
                  bottomRightMotorRpm, feedingRpm, ShooterConfig.RPM_TOLERANCE_FEEDING);
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
                    .addMotor(topLeftMotor, ChassisReference.Clockwise_Positive)
                    .addMotor(topRightMotor, ChassisReference.CounterClockwise_Positive)
                    .addMotor(bottomLeftMotor, ChassisReference.Clockwise_Positive)
                    .addMotor(bottomRightMotor, ChassisReference.CounterClockwise_Positive));

    shooterSimulation.update();
  }

  public boolean currentDetectsGp() {
    return ShooterConfig.GP_DETECT_CURRENT_THRESHOLD
        <= (topRightStatorCurrent
                + topLeftStatorCurrent
                + bottomRightStatorCurrent
                + bottomLeftStatorCurrent)
            / 4.0;
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
    topLeftMotor
        .getConfigurator()
        .apply(
            ShooterConfig.TOP_LEFT_MOTOR_CONFIGS.CurrentLimits.withSupplyCurrentLimit(
                supplyCurrentLimit));
    topRightMotor
        .getConfigurator()
        .apply(
            ShooterConfig.TOP_RIGHT_MOTOR_CONFIG.CurrentLimits.withSupplyCurrentLimit(
                supplyCurrentLimit));
    bottomLeftMotor
        .getConfigurator()
        .apply(
            ShooterConfig.BOTTOM_LEFT_MOTOR_CONFIG.CurrentLimits.withSupplyCurrentLimit(
                supplyCurrentLimit));
    bottomRightMotor
        .getConfigurator()
        .apply(
            ShooterConfig.BOTTOM_RIGHT_MOTOR_CONFIG.CurrentLimits.withSupplyCurrentLimit(
                supplyCurrentLimit));
  }
}
