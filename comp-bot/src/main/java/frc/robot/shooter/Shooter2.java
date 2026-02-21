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

public class Shooter2 extends StateMachineSubsystem<ShooterState2> {
  private static double distanceToScoringRpm(double distance) {
    return FeatureFlags.REGRESSION_MODEL.getAsBoolean()
        ? Shooter2Config.SCORING_REGRESSION_MODEL.calculate(distance)
        : Shooter2Config.DISTANCE_TO_SCORE_RPM.get(distance);
  }

  private static double distanceToFeedingRpm(double distance) {
    return FeatureFlags.REGRESSION_MODEL.getAsBoolean()
        ? Shooter2Config.FEEDING_REGRESSION_MODEL.calculate(distance)
        : Shooter2Config.DISTANCE_TO_FEEDING_RPM.get(distance);
  }

  private final TalonFX topMotor;
  private final TalonFX bottomMotor;

  private final VelocityVoltage velocityRequest =
      new VelocityVoltage(0).withLimitReverseMotion(true).withEnableFOC(true);
  private double scoreDistance = 0;
  private double climbScoreRpm = 0;
  private double feedDistance = 0;
  private double shootingRpm = 0;
  private double feedingRpm = 0;
  private double topMotorRpm = 0;
  private double bottomMotorRpm = 0;

  public Shooter2(TalonFX topMotor, TalonFX bottomMotor) {
    super(SubsystemPriority.SHOOTER, ShooterState2.IDLE);

    topMotor.getConfigurator().apply(Shooter2Config.TOP_MOTOR_CONFIGS);
    bottomMotor.getConfigurator().apply(Shooter2Config.BOTTOM_MOTOR_CONFIG);

    TunablePid.register("Shooter/TopShooter", topMotor, Shooter2Config.TOP_MOTOR_CONFIGS);
    TunablePid.register("Shooter/BottomShooter", bottomMotor, Shooter2Config.BOTTOM_MOTOR_CONFIG);

    this.topMotor = topMotor;
    this.bottomMotor = bottomMotor;
  }

  public void startSelfTestRequest() {
    setStateFromRequest(ShooterState2.SELF_TEST_STOP_MOTORS);
  }

  public void scoreRequest(double distance) {
    this.scoreDistance = distance;
    setStateFromRequest(ShooterState2.SCORE);
  }

  public void climbScoreRequest(boolean isTop) {
    climbScoreRpm = 0.0;
    setStateFromRequest(ShooterState2.CLIMB_SCORE);
  }

  public void feedRequest(double distance) {
    this.feedDistance = distance;
    setStateFromRequest(ShooterState2.FEEDING);
  }

  public void idleRequest() {
    setStateFromRequest(ShooterState2.IDLE);
  }

  @Override
  protected void whileInState(ShooterState2 state) {
    DogLog.log("Shooter/Top/RPM", topMotorRpm);
    DogLog.log("Shooter/Bottom/RPM", bottomMotorRpm);
    DogLog.log("Shooter/GoalShootingRPM", shootingRpm);
    DogLog.log("Shooter/GoalFeedingRPM", feedingRpm);
    DogLog.log("Shooter/AtGoal", atGoal());
    DogLog.log("Shooter/Bottom/Voltage", bottomMotor.getMotorVoltage().getValueAsDouble());
    DogLog.log("Shooter/Top/Voltage", topMotor.getMotorVoltage().getValueAsDouble());

    DogLog.log("Shooter/Top/StatorCurrent", topMotor.getStatorCurrent().getValueAsDouble());
    DogLog.log("Shooter/Bottom/StatorCurrent", bottomMotor.getStatorCurrent().getValueAsDouble());
    DogLog.log("Shooter/Top/SupplyCurrent", topMotor.getSupplyCurrent().getValueAsDouble());
    DogLog.log("Shooter/Bottom/SupplyCurrent", bottomMotor.getSupplyCurrent().getValueAsDouble());

    switch (state) {
      case SCORE -> {
        var setpoint = shootingRpm / 60.0;
        topMotor.setControl(velocityRequest.withVelocity(setpoint));
        bottomMotor.setControl(velocityRequest.withVelocity(setpoint));

        DogLog.log("Shooter/RpmSetpoint", shootingRpm);
      }
      case CLIMB_SCORE -> {
        var setpoint = climbScoreRpm / 60.0;
        topMotor.setControl(velocityRequest.withVelocity(setpoint));
        bottomMotor.setControl(velocityRequest.withVelocity(setpoint));

        DogLog.log("Shooter/RpmSetpoint", climbScoreRpm);
      }
      case FEEDING -> {
        var setpoint = feedingRpm / 60.0;
        topMotor.setControl(velocityRequest.withVelocity(setpoint));
        bottomMotor.setControl(velocityRequest.withVelocity(setpoint));

        DogLog.log("Shooter/RpmSetpoint", feedingRpm);
      }
      case IDLE -> {
        topMotor.disable();
        bottomMotor.disable();
      }
      case SELF_TEST_STOP_MOTORS -> {
        topMotor.stopMotor();
        bottomMotor.stopMotor();
      }
      case SELF_TEST_BOTTOM_MOTOR -> {
        bottomMotor.setVoltage(Shooter2Config.TEST_VOLTAGE);
        topMotor.stopMotor();
      }
      case SELF_TEST_TOP_MOTOR -> {
        topMotor.setVoltage(Shooter2Config.TEST_VOLTAGE);
        bottomMotor.stopMotor();
      }
    }
  }

  @Override
  protected void collectInputs() {
    shootingRpm = Math.min(Shooter2Config.MAX_SAFE_RPM, distanceToScoringRpm(scoreDistance));
    feedingRpm = Math.min(Shooter2Config.MAX_SAFE_RPM, distanceToFeedingRpm(feedDistance));
    topMotorRpm = topMotor.getVelocity().getValueAsDouble() * 60.0;
    bottomMotorRpm = bottomMotor.getVelocity().getValueAsDouble() * 60.0;

    if (getState() == ShooterState2.SELF_TEST_TOP_MOTOR) {
      DogLog.log("Shooter/SelfTest/TopMotor/VelocityGood", MathUtil.isNear(Shooter2Config.SELF_TEST_TOP_MOTOR_EXPECTED_RPM, topMotorRpm, Shooter2Config.SELF_TEST_TOP_MOTOR_RPM_TOLERANCE));
      DogLog.log("Shooter/SelfTest/TopMotor/CurrentGood", MathUtil.isNear(Shooter2Config.SELF_TEST_TOP_MOTOR_EXPECTED_CURRENT, topMotor.getStatorCurrent().getValueAsDouble(), Shooter2Config.SELF_TEST_TOP_MOTOR_CURRENT_TOLERANCE));
    }

    if (getState() == ShooterState2.SELF_TEST_BOTTOM_MOTOR) {
       DogLog.log("Shooter/SelfTest/BottomMotor/VelocityGood", MathUtil.isNear(Shooter2Config.SELF_TEST_BOTTOM_MOTOR_EXPECTED_RPM, topMotorRpm, Shooter2Config.SELF_TEST_BOTTOM_MOTOR_RPM_TOLERANCE));
      DogLog.log("Shooter/SelfTest/BottomMotor/CurrentGood", MathUtil.isNear(Shooter2Config.SELF_TEST_BOTTOM_MOTOR_EXPECTED_CURRENT, bottomMotor.getStatorCurrent().getValueAsDouble(), Shooter2Config.SELF_TEST_BOTTOM_MOTOR_CURRENT_TOLERANCE));
    }
  }

  public boolean atGoal() {
    return switch (getState()) {
      case IDLE -> true;
      case SCORE ->
          MathUtil.isNear(topMotorRpm, shootingRpm, Shooter2Config.RPM_TOLERANCE_SHOOTER)
              && MathUtil.isNear(bottomMotorRpm, shootingRpm, Shooter2Config.RPM_TOLERANCE_SHOOTER);
      case CLIMB_SCORE ->
          MathUtil.isNear(topMotorRpm, climbScoreRpm, Shooter2Config.RPM_TOLERANCE_SHOOTER)
              && MathUtil.isNear(
                  bottomMotorRpm, climbScoreRpm, Shooter2Config.RPM_TOLERANCE_SHOOTER);

      case FEEDING ->
          MathUtil.isNear(topMotorRpm, feedingRpm, Shooter2Config.RPM_TOLERANCE_SHOOTER)
              && MathUtil.isNear(bottomMotorRpm, feedingRpm, Shooter2Config.RPM_TOLERANCE_SHOOTER);
      default -> true;
    };
  }

  @Override
  protected ShooterState2 getNextState(ShooterState2 currentState) {
    return switch (currentState) {
      case SELF_TEST_TOP_MOTOR -> timeout(5) ? ShooterState2.SELF_TEST_STOP_MOTORS : currentState;
      // TODO: This is currently looping between SELF_TEST_STOP_MOTORS and SELF_TEST_BOTTOM_MOTOR infinitely
      case SELF_TEST_STOP_MOTORS ->
          timeout(2) ? ShooterState2.SELF_TEST_BOTTOM_MOTOR : currentState;
      case SELF_TEST_BOTTOM_MOTOR ->
          timeout(5) ? ShooterState2.SELF_TEST_STOP_MOTORS : currentState;
      case SELF_TEST-> currentState;

      default -> currentState;
    };
  }

  @Override
  public void simulationPeriodic() {
    var topshooterSimulation =
        SimKit.velocityMechanism(
            "Shooter/Top",
            (mechanism) ->
                mechanism.addMotor(topMotor, ChassisReference.CounterClockwise_Positive));

    var bottomshooterSimulation =
        SimKit.velocityMechanism(
            "Shooter/Bottom",
            (mechanism) -> mechanism.addMotor(bottomMotor, ChassisReference.Clockwise_Positive));

    topshooterSimulation.update();
    bottomshooterSimulation.update();
  }

  public double getScoreTimeOfFlight(double distance) {
    this.scoreDistance = distance;
    return Shooter2Config.DISTANCE_TO_SCORE_TOF.get(scoreDistance);
  }

  public double getFeedTimeOfFlight(double distance) {
    this.feedDistance = distance;
    return Shooter2Config.DISTANCE_TO_FEED_TOF.get(feedDistance);
  }
}
