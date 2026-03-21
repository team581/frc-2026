package frc.robot.robot_manager.hopper_manager;

import com.team581.util.state_machines.StateMachineSubsystem;
import frc.robot.conveyor.Conveyor;
import frc.robot.deploy.Deploy;
import frc.robot.feeder.Feeder;
import frc.robot.intake.Intake;
import frc.robot.util.scheduling.SubsystemPriority;

public class HopperManager extends StateMachineSubsystem<HopperState> {
  public final Deploy deploy;
  public final Intake intake;
  public final Conveyor conveyor;
  public final Feeder feeder;

  public HopperManager(Deploy deploy, Intake intake, Conveyor conveyor, Feeder feeder) {
    super(SubsystemPriority.HOPPER_MANAGER, HopperState.IDLE);
    this.deploy = deploy;
    this.intake = intake;
    this.conveyor = conveyor;
    this.feeder = feeder;
  }

  @Override
  protected HopperState getNextState(HopperState currentState) {
    return switch (currentState) {
      default -> currentState;
    };
  }

  @Override
  protected void afterTransition(HopperState newState) {
    switch (newState) {
      case IDLE -> {
        // Deploy does nothing in idle
        intake.idleRequest();
        conveyor.idleRequest();
        feeder.idleRequest();
      }
      case SCORE -> {
        deploy.intakeRequest();
        intake.intakeRequest();
        conveyor.shootRequest();
        feeder.shootRequest();
      }
      case INTAKE -> {
        deploy.intakeRequest();
        intake.intakeRequest();
        conveyor.intakeRequest();
        feeder.intakeRequest();
      }
      case SCORE_AND_INTAKE -> {
        deploy.intakeRequest();
        intake.intakeRequest();
        conveyor.shootRequest();
        feeder.shootRequest();
      }
      case CLIMB -> {
        // deploy does nothing
        intake.idleRequest();
        conveyor.idleRequest();
        feeder.idleRequest();
      }
      case REHOME_DEPLOY -> {
        deploy.homingRequest();
        intake.idleRequest();
        conveyor.idleRequest();
        feeder.idleRequest();
      }
    }
  }

  private void setState(HopperState newState) {
    switch (deploy.getState()) {
      case UNHOMED, HOME_INWARD, HOME_OUTWARD -> {}
      default -> setStateFromRequest(newState);
    }
  }

  public void intakeRequest() {
    setStateFromRequest(HopperState.INTAKE);
  }

  public void scoreRequest() {
    if (intake.getState().isIntaking()) {
      setStateFromRequest(HopperState.SCORE_AND_INTAKE);
    }
    setStateFromRequest(HopperState.SCORE);
  }

  public void climbRequest() {
    setStateFromRequest(HopperState.CLIMB);
  }

  public void idleRequest() {
    setStateFromRequest(HopperState.IDLE);
  }

  public void rehomeDeployRequest() {
    setStateFromRequest(HopperState.REHOME_DEPLOY);
  }
}
