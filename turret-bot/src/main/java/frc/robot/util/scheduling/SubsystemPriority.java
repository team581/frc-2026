package frc.robot.util.scheduling;

import com.team581.util.scheduling.SubsystemPriorityBase;

public enum SubsystemPriority implements SubsystemPriorityBase {
  // 20-30 is for manager subsystems
  AUTOS(30),
  ROBOT_MANAGER(29),

  // 10-19 is for sensor subsystems
  LOCALIZATION(11),
  IMU(10),
  // Vision inputs run before localization so that it has fresh vision data for pose estimator
  VISION(10),

  // 0-9 is for actuator subsystems
  SWERVE(0),
  RUMBLE_CONTROLLER(0);

  public final int value;

  private SubsystemPriority(int priority) {
    this.value = priority;
  }

  @Override
  public int getValue() {
    return value;
  }
}
