package frc.robot.util.scheduling;

import com.team581.util.scheduling.SubsystemPriorityBase;

public enum SubsystemPriority implements SubsystemPriorityBase {
  // 20-30 is for manager subsystems
  AUTOS(30),
  ROBOT_MANAGER(29),
  HOPPER_MANAGER(29),
  HEALTH(28),
  POWER_MANAGER(28),

  // 10-19 is for sensor subsystems
  LOCALIZATION(11),
  IMU(10),
  // Vision inputs run before localization so that it has fresh vision data for pose estimator
  VISION(10),
  HUB_ACTIVITY(10),

  // 0-9 is for actuator subsystems
  DEPLOY(0),
  SWERVE(0),
  SHOOTER_HOOD(0),
  INTAKE(0),
  SHOOTER(0),
  RUMBLE_CONTROLLER(0),
  KICKER(0),
  FEEDER(0),
  CONVEYOR(0);

  public final int value;

  SubsystemPriority(int priority) {
    this.value = priority;
  }

  @Override
  public int getValue() {
    return value;
  }
}
