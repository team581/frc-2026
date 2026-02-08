package frc.robot.deploy;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;

public enum DeployState {
  INTAKE(0.0),
  STOWED(0),
  SHOOTING(0.0),
  HOMING(0),
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
