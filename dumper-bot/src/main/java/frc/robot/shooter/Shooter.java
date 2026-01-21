package frc.robot.shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team581.simkit.SimKit;
import com.team581.util.state_machines.StateMachineSubsystem;
import com.team581.util.tuning.TunablePid;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import frc.robot.util.scheduling.SubsystemPriority;
import java.util.Map;

public class Shooter extends StateMachineSubsystem<ShooterState> {
  private static final int RPM_TOLERANCE_KICKER = 100;

  private static final int RPM_TOLERANCE_SHOOTER = 50;

  private static final double MAX_SAFE_RPM = 4000;

  private static final InterpolatingDoubleTreeMap DISTANCE_TO_SCORE_RPM =
      InterpolatingDoubleTreeMap.ofEntries(
          Map.entry(Units.inchesToMeters(57.0), 2250.0), Map.entry(3.376, 3300.0));
  private static final InterpolatingDoubleTreeMap DISTANCE_TO_FEEDING_RPM =
      InterpolatingDoubleTreeMap.ofEntries(
          Map.entry(1.0, 2000.0), Map.entry(2.0, 3500.0), Map.entry(5.0, 5000.0));
  private static final InterpolatingDoubleTreeMap DISTANCE_TO_SCORE_TOF =
      InterpolatingDoubleTreeMap.ofEntries(Map.entry(Units.inchesToMeters(57.0), 0.0));

  private final TalonFX leftMotor;
  private final TalonFX rightMotor;
  private final TalonFX kickerMotor;

  private final VelocityVoltage voltageRequest = new VelocityVoltage(0).withEnableFOC(true);

  private double hubDistance = 0;
  private double feedDistance = 0;

  private double shootingRpm = 0;
  private double feedingRpm = 0;
  private double leftMotorRpm = 0;
  private double rightMotorRpm = 0;
  private double kickerMotorRpm = 0;

  public Shooter(TalonFX leftMotor, TalonFX rightMotor, TalonFX kickerMotor) {
    super(SubsystemPriority.SHOOTER, ShooterState.IDLE);

    var leftConfigs =
        new TalonFXConfiguration()
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(1))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(100.0)
                    .withMotionMagicAcceleration(20.0))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimitEnable(true)
                    .withStatorCurrentLimitEnable(true)
                    .withStatorCurrentLimit(100)
                    .withSupplyCurrentLimit(100))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Coast)
                    .withInverted(InvertedValue.CounterClockwise_Positive))
            .withSlot0(new Slot0Configs().withKP(0.13).withKV(0.132).withKS(0.0).withKA(0.0))
            .withTorqueCurrent(
                new TorqueCurrentConfigs()
                    .withPeakForwardTorqueCurrent(200)
                    .withPeakReverseTorqueCurrent(0));
    var rightConfigs =
        new TalonFXConfiguration()
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(1))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(100.0)
                    .withMotionMagicAcceleration(20.0))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimitEnable(true)
                    .withStatorCurrentLimitEnable(true)
                    .withStatorCurrentLimit(100)
                    .withSupplyCurrentLimit(100))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Coast)
                    .withInverted(InvertedValue.Clockwise_Positive))
            .withSlot0(new Slot0Configs().withKP(0.13).withKV(0.132).withKS(0.0).withKA(0.0))
            .withTorqueCurrent(
                new TorqueCurrentConfigs()
                    .withPeakForwardTorqueCurrent(200)
                    .withPeakReverseTorqueCurrent(0));

    var kickerConfigs =
        new TalonFXConfiguration()
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(1))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(100.0)
                    .withMotionMagicAcceleration(20.0))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimitEnable(true)
                    .withStatorCurrentLimitEnable(true)
                    .withStatorCurrentLimit(100)
                    .withSupplyCurrentLimit(100))
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast))
            .withSlot0(new Slot0Configs().withKP(0.0).withKV(0.0).withKS(0.0))
            .withTorqueCurrent(
                new TorqueCurrentConfigs()
                    .withPeakForwardTorqueCurrent(200)
                    .withPeakReverseTorqueCurrent(0));

    leftMotor.getConfigurator().apply(leftConfigs);
    rightMotor.getConfigurator().apply(rightConfigs);
    kickerMotor.getConfigurator().apply(kickerConfigs);

    TunablePid.register("Shooter/LeftShooter", leftMotor, leftConfigs);
    TunablePid.register("Shooter/RightShooter", rightMotor, rightConfigs);
    TunablePid.register("Shooter/RightKicker", kickerMotor, kickerConfigs);

    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
    this.kickerMotor = kickerMotor;
  }

  public void scoreRequest(double distance) {
    this.hubDistance = distance;
    setStateFromRequest(ShooterState.SCORE);
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
    DogLog.log("Shooter/Kicker/RPM", kickerMotorRpm);
    DogLog.log("Shooter/GoalShootingRPM", shootingRpm);
    DogLog.log("Shooter/GoalFeedingRPM", feedingRpm);
    DogLog.log("Shooter/AtGoal", atGoal());
    //   DogLog.log("Shooter/Right/StatorCurrent",
    // rightMotor.getStatorCurrent().getValueAsDouble());
    //   DogLog.log("Shooter/Left/StatorCurrent", leftMotor.getStatorCurrent().getValueAsDouble());
    //   DogLog.log("Shooter/Kicker/StatorCurrent",
    // kickerMotor.getStatorCurrent().getValueAsDouble());
    DogLog.log("Shooter/Right/Voltage", rightMotor.getMotorVoltage().getValueAsDouble());
    DogLog.log("Shooter/Left/Voltage", leftMotor.getMotorVoltage().getValueAsDouble());
    DogLog.log("Shooter/Kicker/Voltage", kickerMotor.getMotorVoltage().getValueAsDouble());

    switch (state) {
      case SCORE -> {
        var setpoint = shootingRpm / 60.0;
        leftMotor.setControl(voltageRequest.withVelocity(setpoint));
        rightMotor.setControl(voltageRequest.withVelocity(setpoint));
        kickerMotor.setControl(voltageRequest.withVelocity(setpoint));
        DogLog.log("Shooter/RpmSetpoint", shootingRpm);
      }
      case FEEDING -> {
        var setpoint = feedingRpm / 60.0;
        leftMotor.setControl(voltageRequest.withVelocity(setpoint));
        rightMotor.setControl(voltageRequest.withVelocity(setpoint));
        kickerMotor.setControl(voltageRequest.withVelocity(setpoint));
        DogLog.log("Shooter/RpmSetpoint", feedingRpm);
      }
      case IDLE -> {
        leftMotor.disable();
        rightMotor.disable();
        kickerMotor.disable();
        DogLog.log("Shooter/RpmSetpoint", -1);
      }
    }
  }

  @Override
  protected void collectInputs() {
    shootingRpm = Math.min(MAX_SAFE_RPM, DISTANCE_TO_SCORE_RPM.get(hubDistance));
    feedingRpm = Math.min(MAX_SAFE_RPM, DISTANCE_TO_FEEDING_RPM.get(feedDistance));

    leftMotorRpm = leftMotor.getVelocity().getValueAsDouble() * 60.0;
    rightMotorRpm = rightMotor.getVelocity().getValueAsDouble() * 60.0;
    kickerMotorRpm = kickerMotor.getVelocity().getValueAsDouble() * 60.0;
  }

  public boolean atGoal() {
    return switch (getState()) {
      case IDLE -> true;
      case SCORE ->
          MathUtil.isNear(leftMotorRpm, shootingRpm, RPM_TOLERANCE_SHOOTER)
              && MathUtil.isNear(rightMotorRpm, shootingRpm, RPM_TOLERANCE_SHOOTER)
              && MathUtil.isNear(kickerMotorRpm, shootingRpm, RPM_TOLERANCE_KICKER);
      case FEEDING ->
          MathUtil.isNear(leftMotorRpm, feedingRpm, RPM_TOLERANCE_SHOOTER)
              && MathUtil.isNear(rightMotorRpm, feedingRpm, RPM_TOLERANCE_SHOOTER)
              && MathUtil.isNear(kickerMotorRpm, feedingRpm, RPM_TOLERANCE_KICKER);
    };
  }

  @Override
  public void simulationPeriodic() {
    var shooterSimulation =
        SimKit.velocityMechanism(
            "shooter",
            (mechanism) ->
                mechanism
                    .addMotor(leftMotor)
                    .addMotor(rightMotor)
                    .addMotor(kickerMotor)
                    .withMinVelocity(0));

    shooterSimulation.update();
  }

  public double getCurrentTimeOfFlight() {
    return switch (getState()) {
      case SCORE -> DISTANCE_TO_SCORE_TOF.get(feedDistance);
      case FEEDING -> DISTANCE_TO_FEEDING_RPM.get(feedDistance);
      default -> 0.0;
    };
  }
}
