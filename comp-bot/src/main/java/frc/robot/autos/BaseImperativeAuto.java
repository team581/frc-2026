package frc.robot.autos;

import com.team581.autos.BaseAuto;
import com.team581.controller.ControllerHelpers;
import com.team581.trailblazer.Trailblazer;
import com.team581.util.state_machines.StateMachine;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.robot_manager.RobotManager;

public abstract class BaseImperativeAuto<S extends Enum<S>> extends StateMachine<S>
    implements BaseAuto {
  private static final double INPUT_THRESHOLD = 0.4;

  protected final RobotManager robotManager;
  protected final Trailblazer trailblazer;

  /** If the robot has ever been enabled during autonomous. */
  protected boolean hasRanAuto = false;

  protected boolean hasSeenInputInTeleop = false;

  protected BaseImperativeAuto(S initialState, RobotManager robotManager, Trailblazer trailblazer) {
    super(initialState);
    this.robotManager = robotManager;
    this.trailblazer = trailblazer;
  }

  @Override
  public boolean shouldRun() {
    // If we're in autonomous, always run the auto
    if (DriverStation.isAutonomous()) {
      return true;
    }
    // If we have run an auto previously, then continue running it until we see teleop inputs

    // Otherwise, we are in teleop but haven't run the auto previously, so we should stay in teleop
    return false;
  }

  @Override
  protected void collectInputs() {
    // Keep track if we have ever been autonomous enabled
    hasRanAuto = hasRanAuto || DriverStation.isAutonomousEnabled();

    // Input has to be seen in teleop again if we ever go into autonomous
    if (DriverStation.isAutonomous()) {
      hasSeenInputInTeleop = false;
    }

    // Keep track if driver has put any meaningful input during teleop enabled
    if (DriverStation.isTeleopEnabled()) {
      hasSeenInputInTeleop =
          hasSeenInputInTeleop
              || ControllerHelpers.getJoystickMagnitude(
                      robotManager.driverController.getLeftX(),
                      robotManager.driverController.getLeftY(),
                      1.0)
                  > INPUT_THRESHOLD
              || Math.abs(robotManager.driverController.getRightX()) > INPUT_THRESHOLD;
    }
  }
}
