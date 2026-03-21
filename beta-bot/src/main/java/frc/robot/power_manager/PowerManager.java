package frc.robot.power_manager;

import com.team581.mechanisms.PowerManaged;
import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import frc.robot.deploy.Deploy;
import frc.robot.dye_rotor.DyeRotor;
import frc.robot.intake.GenericIntake;
import frc.robot.shooter.Shooter;
import frc.robot.shooter_hood.ShooterHood;
import frc.robot.swerve.Swerve;
import frc.robot.turret.Turret;
import frc.robot.util.scheduling.SubsystemPriority;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class PowerManager extends StateMachineSubsystem<PowerManagerState> {
  private final ExecutorService executor = Executors.newSingleThreadExecutor();
  private final PowerManaged shooterHood;
  private final PowerManaged swerve;
  private final PowerManaged shooter;
  private final PowerManaged dyeRotor;
  private final PowerManaged turret;
  private final PowerManaged intake;
  private final PowerManaged deploy;

  public PowerManager(
      ShooterHood shooterHood,
      Swerve swerve,
      Shooter shooter,
      DyeRotor dyeRotor,
      Turret turret,
      GenericIntake intake,
      Deploy deploy) {
    super(SubsystemPriority.POWER_MANAGER, PowerManagerState.IDLE);

    this.shooterHood = shooterHood;
    this.swerve = swerve;
    this.shooter = shooter;
    this.dyeRotor = dyeRotor;
    this.turret = turret;
    this.intake = intake;
    this.deploy = deploy;
  }

  public void idleRequest() {
    setStateFromRequest(PowerManagerState.IDLE);
  }

  public void shootingRequest() {
    setStateFromRequest(PowerManagerState.SHOOTING);
  }

  @Override
  protected void afterTransition(PowerManagerState newState) {
    executor.execute(
        () -> {
          DogLog.timestamp("PowerManager/UpdatedCurrentsAt");
          shooterHood.applyCurrentLimits(newState.shooterHoodSupplyCurrent);
          swerve.applyCurrentLimits(newState.swerveSupplyCurrent);
          shooter.applyCurrentLimits(newState.shooterSupplyCurrent);
          dyeRotor.applyCurrentLimits(newState.dyeRotorSupplyCurrent);
          turret.applyCurrentLimits(newState.turretSupplyCurrent);
          intake.applyCurrentLimits(newState.intakeSupplyCurrent);
          deploy.applyCurrentLimits(newState.deploySupplyCurrent);
        });
  }
}
