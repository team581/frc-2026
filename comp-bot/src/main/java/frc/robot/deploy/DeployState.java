package frc.robot.deploy;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;

public enum DeployState {
  INTAKE(DeployConfig.MAX_LENGTH - 0.25),
  STOW(1.0),
  HOPPER_SHUFFLING_OUT(INTAKE.getLength()),
  HOPPER_SHUFFLING_IN(INTAKE.getLength() - DeployConfig.HOPPER_SHUFFLE_DISTANCE),
  HOME_INWARD(0),
  HOME_OUTWARD(DeployConfig.MAX_LENGTH),
  UNHOMED(0);

  private final DoubleSubscriber tunableLength;

  private DeployState(double length) {

    this.tunableLength = DogLog.tunable("Deploy/State/" + name(), length);
  }

  public double getLength() {
    return tunableLength.get();
  }
}
