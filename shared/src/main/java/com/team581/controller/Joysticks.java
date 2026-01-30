package com.team581.controller;

import com.team581.util.scheduling.SubsystemPriorityBase;
import com.team581.util.state_machines.StateMachineSubsystem;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;

public class Joysticks extends StateMachineSubsystem<JoystickState> {
  private final XboxController controller;
  private final double INPUT_THRESHOLD = 0.1;

  private boolean inputting = false;

  public Joysticks(SubsystemPriorityBase priority, XboxController controller) {
    super(priority, JoystickState.NOT_SEEN_INPUT_TELEOP);
    this.controller = controller;
  }

  @Override
  protected JoystickState getNextState(JoystickState currentState) {
      return switch (currentState) {
        case NOT_SEEN_INPUT_TELEOP -> DriverStation.isTeleop() && inputting ? JoystickState.SEEN_INPUT_TELEOP : currentState;
        case SEEN_INPUT_TELEOP -> currentState;
      };
  }

  @Override
  protected void collectInputs() {
    var leftX = controller.getLeftX();
    var leftY = controller.getLeftY();
    var rightX = controller.getRightX();

    var overLeftThreshold = ControllerHelpers.getJoystickMagnitude(leftX, leftY, 1.0) > INPUT_THRESHOLD;
    var overRightThreshold = Math.abs(rightX) > INPUT_THRESHOLD;

    inputting = overLeftThreshold || overRightThreshold;
  }

  public boolean hasInputInTeleop() {
    return getState().value;
  }

  @Override
  public void teleopInit() {
      setStateFromRequest(JoystickState.NOT_SEEN_INPUT_TELEOP);
  }
}
