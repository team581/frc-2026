package frc.robot.deploy;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;

public enum DeployState {
  INTAKE(DeployConfig.MAX_LENGTH),
  STOW(1.0),
  HOPPER_SHUFFLING_OUT(INTAKE.getLength()),
  HOPPER_SHUFFLING_IN(INTAKE.getLength() - DeployConfig.HOPPER_SHUFFLE_DISTANCE),
  HOME(0),
  UNHOMED(0);

  private final DoubleSubscriber tunableLength;

  private DeployState(double length) {

    this.tunableLength = DogLog.tunable("Deploy/State/" + name(), length);
  }

  public double getLength() {
    return tunableLength.get();
  }
}
