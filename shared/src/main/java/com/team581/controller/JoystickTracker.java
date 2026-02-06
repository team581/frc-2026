package com.team581.controller;

import com.team581.util.scheduling.SubsystemPriorityBase;
import com.team581.util.state_machines.StateMachineSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import java.util.function.BooleanSupplier;

public class JoystickTracker extends StateMachineSubsystem<JoystickState> {
  private final XboxController controller;
  private static final double INPUT_THRESHOLD = 0.4;
  private final BooleanSupplier doAutoSafeCheck;

  private boolean inputting = false;
  private boolean beenInAuto = false;

  public JoystickTracker(
      SubsystemPriorityBase priority, XboxController controller, BooleanSupplier doAutoSafeCheck) {
    super(priority, JoystickState.NOT_SEEN_INPUT_TELEOP);
    this.controller = controller;
    this.doAutoSafeCheck = doAutoSafeCheck;
  }

  @Override
  protected JoystickState getNextState(JoystickState currentState) {
    return switch (currentState) {
      /*
      Check if it is inputting during teleop, change to SEEN_INPUT_TELEOP. if requires for the
      robot to have been in auto, automatically set to SEEN_INPUT_TELEOP to avoid starting an
      auto during teleop
      */
      case NOT_SEEN_INPUT_TELEOP ->
          (DriverStation.isTeleop() && inputting) || (!beenInAuto && doAutoSafeCheck.getAsBoolean())
              ? JoystickState.SEEN_INPUT_TELEOP
              : currentState;
      // If it is autonomous enabled or teleop disabled reset to NOT_SEEN_INPUT_TELEOP
      case SEEN_INPUT_TELEOP ->
          DriverStation.isAutonomousEnabled()
                  || (DriverStation.isTeleop() && DriverStation.isDisabled())
              ? JoystickState.NOT_SEEN_INPUT_TELEOP
              : currentState;
    };
  }

  @Override
  protected void collectInputs() {
    if (DriverStation.isAutonomous()) {
      beenInAuto = DriverStation.isEnabled();
    }
    var leftX = controller.getLeftX();
    var leftY = controller.getLeftY();
    var rightX = controller.getRightX();

    var overLeftThreshold =
        ControllerHelpers.getJoystickMagnitude(leftX, leftY, 1.0) > INPUT_THRESHOLD;
    var overRightThreshold = Math.abs(rightX) > INPUT_THRESHOLD;

    inputting = overLeftThreshold || overRightThreshold;
  }

  public boolean hasInputInTeleop() {
    return getState() == JoystickState.SEEN_INPUT_TELEOP;
  }
}
