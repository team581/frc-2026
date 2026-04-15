package frc.robot.robot_manager.hopper_manager;

public enum HopperState {
  IDLE_DEPLOYED(true),
  IDLE_STOWED(false),
  INTAKING(true),
  EJECTING(false),
  UNJAMMING(false),
  AUTO_UNJAM(false),
  SHOOT(false),
  SHOOT_AND_INTAKE(false);

  public final boolean canBallFill;

  HopperState(boolean canBallFill) {
    this.canBallFill = canBallFill;
  }
}
