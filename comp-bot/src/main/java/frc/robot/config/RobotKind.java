package frc.robot.config;

import edu.wpi.first.wpilibj.RobotController;

public enum RobotKind {
  PRACTICE_BOT,
  COMP_BOT;

  public static final boolean IS_COMP_BOT = fromSerialNumber() == COMP_BOT;

  /** Returns the RobotKind by matching the serial number to a known RobotKind. */
  public static RobotKind fromSerialNumber() {
    return switch (RobotController.getSerialNumber()) {
      case "0322443D" -> PRACTICE_BOT;
      default -> COMP_BOT;
    };
  }

  RobotKind() {}
}
