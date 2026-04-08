package frc.robot.deploy;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;

public enum DeployState {
  INTAKE(DeployConfig.MAX_LENGTH),
  STOW(1.0),
  // placeholder values
  HOPPER_COMPACTION_IN(3.0),
  HOME_INWARD(DeployConfig.MIN_LENGTH),
  HOME_OUTWARD(DeployConfig.MAX_LENGTH),
  UNHOMED(0);

  private final DoubleSubscriber tunableLength;

  DeployState(double length) {

    this.tunableLength = DogLog.tunable("Deploy/State/" + name(), length);
  }

  public boolean isHoming() {
    return switch (this) {
      case UNHOMED, HOME_INWARD, HOME_OUTWARD -> true;
      default -> false;
    };
  }

  public double getLength() {
    return tunableLength.get();
  }
}
