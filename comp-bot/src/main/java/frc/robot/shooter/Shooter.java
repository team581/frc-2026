package frc.robot.shooter;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.team581.simkit.SimKit;
import com.team581.util.state_machines.StateMachineSubsystem;
import com.team581.util.tuning.TunablePid;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import frc.robot.config.FeatureFlags;
import frc.robot.util.scheduling.SubsystemPriority;

public class Shooter extends StateMachineSubsystem<ShooterState> {
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

    DogLog.log("Shooter/Left/StatorCurrent", leftMotor.getStatorCurrent().getValueAsDouble());
    DogLog.log("Shooter/Right/StatorCurrent", rightMotor.getStatorCurrent().getValueAsDouble());
    DogLog.log("Shooter/Left/SupplyCurrent", leftMotor.getSupplyCurrent().getValueAsDouble());
    DogLog.log("Shooter/Right/SupplyCurrent", rightMotor.getSupplyCurrent().getValueAsDouble());

    switch (state) {
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
      case IDLE -> {
        leftMotor.disable();
        rightMotor.disable();

        DogLog.log("Shooter/RpmSetpoint", -1.0);
      }
    }
  }

  @Override
  protected void collectInputs() {
    shootingRpm = Math.min(ShooterConfig.MAX_SAFE_RPM, distanceToScoringRpm(scoreDistance));
    feedingRpm = Math.min(ShooterConfig.MAX_SAFE_RPM, distanceToFeedingRpm(feedDistance));

    leftMotorRpm = leftMotor.getVelocity().getValueAsDouble() * 60.0;
    rightMotorRpm = rightMotor.getVelocity().getValueAsDouble() * 60.0;
  }

  public boolean atGoal() {
    return switch (getState()) {
      case IDLE -> true;
      case SCORE ->
          MathUtil.isNear(leftMotorRpm, shootingRpm, ShooterConfig.RPM_TOLERANCE_SHOOTER)
              && MathUtil.isNear(rightMotorRpm, shootingRpm, ShooterConfig.RPM_TOLERANCE_SHOOTER);
      case CLIMB_SCORE ->
          MathUtil.isNear(leftMotorRpm, climbScoreRpm, ShooterConfig.RPM_TOLERANCE_SHOOTER)
              && MathUtil.isNear(rightMotorRpm, climbScoreRpm, ShooterConfig.RPM_TOLERANCE_SHOOTER);

      case FEEDING ->
          MathUtil.isNear(leftMotorRpm, feedingRpm, ShooterConfig.RPM_TOLERANCE_SHOOTER)
              && MathUtil.isNear(rightMotorRpm, feedingRpm, ShooterConfig.RPM_TOLERANCE_SHOOTER);
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

  public double getScoreTimeOfFlight(double distance) {
    this.scoreDistance = distance;
    return ShooterConfig.DISTANCE_TO_SCORE_TOF.get(scoreDistance);
  }

  public double getFeedTimeOfFlight(double distance) {
    this.feedDistance = distance;
    return ShooterConfig.DISTANCE_TO_FEED_TOF.get(feedDistance);
  }
}
