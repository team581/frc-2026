package frc.robot.power_manager;

public enum PowerManagerState {
  IDLE(10, 50, 100, 30, 10, 60, 18),
  SHOOTING(10, 50, 100, 80, 20, 30, 5);

  final double shooterHoodSupplyCurrent;
  final double swerveSupplyCurrent;
  final double shooterSupplyCurrent;
  final double dyeRotorSupplyCurrent;
  final double turretSupplyCurrent;
  final double intakeSupplyCurrent;
  final double deploySupplyCurrent;

  PowerManagerState(
      double shooterHoodSupplyCurrent,
      double swerveSupplyCurrent,
      double shooterSupplyCurrent,
      double dyeRotorSupplyCurrent,
      double turretSupplyCurrent,
      double intakeSupplyCurrent,
      double deploySupplyCurrent) {
    this.shooterHoodSupplyCurrent = shooterHoodSupplyCurrent;
    this.swerveSupplyCurrent = swerveSupplyCurrent;
    this.shooterSupplyCurrent = shooterSupplyCurrent;
    this.dyeRotorSupplyCurrent = dyeRotorSupplyCurrent;
    this.turretSupplyCurrent = turretSupplyCurrent;
    this.intakeSupplyCurrent = intakeSupplyCurrent;
    this.deploySupplyCurrent = deploySupplyCurrent;
  }
}
