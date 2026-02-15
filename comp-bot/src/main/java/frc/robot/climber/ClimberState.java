package frc.robot.climber;

public enum ClimberState {
  STOWED(0),
  L1_LINEUP(10),
  L1_HANG(30),
  L2_LINEUP(10),
  L2_HANG(30),
  L3_LINEUP(10),
  L3_HANG(30);

  public final double height;

  ClimberState(double height) {
    this.height = height;
  }
}
