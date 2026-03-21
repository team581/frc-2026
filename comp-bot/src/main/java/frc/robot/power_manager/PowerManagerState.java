package frc.robot.power_manager;

public enum PowerManagerState {
  IDLE(100, 60, 50, 18, 10, 30, 50, 50, 50, 40),
  SHOOTING(100, 60, 50, 18, 10, 30, 50, 50, 50, 10);

  final double shooterHoodSupplyCurrent;
  final double swerveSupplyCurrent;
  final double shooterSupplyCurrent;
  final double dyeRotorSupplyCurrent;
  final double intakeSupplyCurrent;
  final double deploySupplyCurrent;
  final double climberSupplyCurrent;
  final double kickerSupplyCurrent;
  final double feederSupplyCurrent;
  final double conveyorSupplyCurrent;

  PowerManagerState(
      double shooterSupplyCurrent,
      double dyeRotorSupplyCurrent,
      double intakeSupplyCurrent,
      double deploySupplyCurrent,
      double shooterHoodSupplyCurrent,
      double climberSupplyCurrent,
      double kickerSupplyCurrent,
      double feederSupplyCurrent,
      double conveyorSupplyCurrent,
      double swerveSupplyCurrent) {
    this.shooterSupplyCurrent = shooterSupplyCurrent;
    this.dyeRotorSupplyCurrent = dyeRotorSupplyCurrent;
    this.intakeSupplyCurrent = intakeSupplyCurrent;
    this.deploySupplyCurrent = deploySupplyCurrent;
    this.shooterHoodSupplyCurrent = shooterHoodSupplyCurrent;
    this.climberSupplyCurrent = climberSupplyCurrent;
    this.kickerSupplyCurrent = kickerSupplyCurrent;
    this.feederSupplyCurrent = feederSupplyCurrent;
    this.conveyorSupplyCurrent = conveyorSupplyCurrent;
    this.swerveSupplyCurrent = swerveSupplyCurrent;
  }
}
