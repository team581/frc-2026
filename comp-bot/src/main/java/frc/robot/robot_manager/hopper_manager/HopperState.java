package frc.robot.robot_manager.hopper_manager;

public enum HopperState {
  IDLE,
  INTAKE,
  CLIMB,
  SCORE,
  SCORE_AND_INTAKE,
  REHOME_DEPLOY;

  public boolean isIntaking() {
    return this == INTAKE || this == SCORE_AND_INTAKE;
  }
}
