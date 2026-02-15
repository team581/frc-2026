package frc.robot.deploy;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;

public enum DeployState {
  INTAKE(DeployConfig.MAX_LENGTH),
  STOW(1.0),
  HOPPER_SHUFFLING(INTAKE.getLength()),
  HOME(0),
  UNHOMED(0),
  CATCHUP_TO_LEFT(0),
  CATCHUP_TO_RIGHT(0);

  private final DoubleSubscriber tunableLength;

  private DeployState(double length) {

    this.tunableLength = DogLog.tunable("Deploy/State/" + name(), length);
  }

  public double getLength() {
    return tunableLength.get();
  }
}
