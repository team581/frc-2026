package frc.robot.health;

import com.team581.mechanisms.vision.CameraHealth;
import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.config.DSOptions;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.vision.limelight.Limelight;

public class HealthManager extends StateMachineSubsystem<HealthState> {
  private final Limelight turretLimelight;
  private final Limelight backLimelight;

  private boolean localizationHealthy = true;
  private boolean fuelDetectionHealthy = true;
  private boolean allCamerasHealthy = true;

  // TODO: Add intake Limelight
  public HealthManager(Limelight turretLimelight, Limelight backLimelight) {
    super(SubsystemPriority.HEALTH, HealthState.DEFAULT_STATE);

    this.turretLimelight = turretLimelight;
    this.backLimelight = backLimelight;
  }

  /** Returns whether all cameras are healthy. */
  public boolean isAllCamerasHealthy() {
    return allCamerasHealthy;
  }

  /** Returns whether the robot's ability to detect fuel is healthy. */
  public boolean isFuelDetectionHealthy() {
    return fuelDetectionHealthy;
  }

  /** Returns whether the robot's ability to localize itself is healthy. */
  public boolean isLocalizationHealthy() {
    return localizationHealthy && DSOptions.USE_TAG_LIMELIGHTS.getAsBoolean();
  }

  @Override
  protected void collectInputs() {
    localizationHealthy =
        RobotBase.isSimulation() || turretLimelight.getCameraHealth() != CameraHealth.OFFLINE;
    // TODO: This should use intake limelight
    fuelDetectionHealthy =
        RobotBase.isSimulation() || backLimelight.getCameraHealth() != CameraHealth.OFFLINE;
    allCamerasHealthy = localizationHealthy && fuelDetectionHealthy;
  }

  @Override
  protected void whileInState(HealthState state) {
    DogLog.log("Health/LocalizationHealthy", localizationHealthy);
    DogLog.log("Health/FuelDetectionHealthy", fuelDetectionHealthy);
    DogLog.log("Health/AllCamerasHealthy", allCamerasHealthy);
  }
}
