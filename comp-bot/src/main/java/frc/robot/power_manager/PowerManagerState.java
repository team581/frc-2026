package frc.robot.power_manager;

public enum PowerManagerState {
  AUTO_FIRST_SEGMENT(25, 40, 15, 40, 10, 10, 60),
  IDLE(10, 25, 15, 40, 10, 10, 40),
  SCORING(25, 20, 15, 40, 20, 20, 40),
  SCORING_FAR(25, 20, 15, 40, 10, 10, 40),
  FEEDING(35, 20, 15, 40, 15, 15, 40),
  TURBO_MODE(25, 5, 18, 10, 10, 10, 60);

  final double shooterSupplyCurrent;
  final double intakeSupplyCurrent;
  final double deploySupplyCurrent;
  final double shooterHoodSupplyCurrent;
  final double feederSupplyCurrent;
  final double conveyorSupplyCurrent;
  final double swerveSupplyCurrent;

  PowerManagerState(
      double shooterSupplyCurrent,
      double intakeSupplyCurrent,
      double deploySupplyCurrent,
      double shooterHoodSupplyCurrent,
      double feederSupplyCurrent,
      double conveyorSupplyCurrent,
      double swerveSupplyCurrent) {
    this.shooterSupplyCurrent = shooterSupplyCurrent;
    this.intakeSupplyCurrent = intakeSupplyCurrent;
    this.deploySupplyCurrent = deploySupplyCurrent;
    this.shooterHoodSupplyCurrent = shooterHoodSupplyCurrent;
    this.feederSupplyCurrent = feederSupplyCurrent;
    this.conveyorSupplyCurrent = conveyorSupplyCurrent;
    this.swerveSupplyCurrent = swerveSupplyCurrent;
  }
}
