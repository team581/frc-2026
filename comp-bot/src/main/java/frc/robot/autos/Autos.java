package frc.robot.autos;

import com.team581.autos.AutoChooser;
import com.team581.config.DSOption;
import com.team581.swerve.DriveSourceType;
import com.team581.trailblazer.Trailblazer;
import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.config.DSOptions;
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
  protected void afterTransition(AutoSelection newState) {
    // Recreate the auto instances when the selection changes
    selectedAuto = newState.auto.apply(robotManager, trailblazer);
  }

  @Override
  protected AutoSelection getNextState(AutoSelection currentState) {
    if (DriverStation.isEnabled()) {
      return currentState;
    }

    return autoChooser.getSelectedAuto();
  }

  @Override
  protected void whileInState(AutoSelection state) {
    if (DriverStation.isDisabled()) {
      if (!hasEnabledAuto && DSOptions.RESET_POSE_FOR_AUTO.getAsBoolean()
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

    if (selectedAuto.shouldRun()) {
      selectedAuto.beforePeriodic();
      selectedAuto.periodic();
      robotManager.swerve.setDriveSourceType(DriveSourceType.FIELD_CENTRIC_CLOSED_LOOP);
      DogLog.log("Autos/ShouldRun", true);
    } else {
      // Restore teleop drive source
      robotManager.swerve.setDriveSourceType(DriveSourceType.DRIVER_PERSPECTIVE_OPEN_LOOP);
      DogLog.log("Autos/ShouldRun", false);
    }
  }
}
