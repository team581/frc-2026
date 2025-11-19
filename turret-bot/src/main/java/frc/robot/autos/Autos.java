package frc.robot.autos;

import com.team581.autos.AutoChooser;
import com.team581.trailblazer.Trailblazer;
import com.team581.util.state_machines.StateMachineSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.robot_manager.RobotManager;
import frc.robot.util.scheduling.SubsystemPriority;

public class Autos extends StateMachineSubsystem<AutoSelection> {
  private final AutoChooser<AutoSelection> autoChooser;
  private final RobotManager robotManager;
  private final Trailblazer trailblazer;
  private boolean hasEnabledAuto = false;
  private BaseImperativeAuto<?> selectedAuto;

  public Autos(RobotManager robotManager, Trailblazer trailblazer) {
    super(SubsystemPriority.AUTOS, AutoSelection.DO_NOTHING);

    autoChooser = new AutoChooser<>(AutoSelection.values(), AutoSelection.DO_NOTHING);
    this.robotManager = robotManager;
    this.trailblazer = trailblazer;

    selectedAuto = AutoSelection.DO_NOTHING.auto.apply(robotManager, trailblazer);
  }

  @Override
  protected AutoSelection getNextState(AutoSelection currentState) {
    if (DriverStation.isEnabled()) {
      return currentState;
    }

    return autoChooser.getSelectedAuto();
  }

  @Override
  protected void afterTransition(AutoSelection newState) {
    // Recreate the auto instances when the selection changes
    selectedAuto = newState.auto.apply(robotManager, trailblazer);
  }

  @Override
  protected void whileInState(AutoSelection state) {
    if (DriverStation.isDisabled()) {
      if (!hasEnabledAuto
          && (RobotBase.isSimulation()
              || DriverStation.isAutonomous()
              || DriverStation.isFMSAttached())) {
        // Continuously reset pose
        robotManager.localization.resetPose(selectedAuto.getStartingPoint().getPose());
      }
    }

    if (DriverStation.isAutonomousEnabled()) {
      hasEnabledAuto = true;
    }

    if (DriverStation.isAutonomous()) {
      selectedAuto.beforePeriodic();
      selectedAuto.periodic();
    }
  }
}
