package frc.robot.deploy;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;

public enum DeployState {
  INTAKE(DeployConfig.MAX_LENGTH - 0.25),
  STOW(1.0),
  HOPPER_COMPACTION_IN(10.5),
  HOPPER_COMPACTION_FINISH(7.5),
  HOME_INWARD(DeployConfig.MIN_LENGTH),
  HOME_OUTWARD(DeployConfig.MAX_LENGTH),
  UNHOMED(0);

  private final DoubleSubscriber tunableLength;

  DeployState(double length) {

    this.tunableLength = DogLog.tunable("Deploy/State/" + name(), length);
  }

  public double getLength() {
    return tunableLength.get();
  }
}
