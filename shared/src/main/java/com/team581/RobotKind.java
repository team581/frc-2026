package com.team581;

import edu.wpi.first.wpilibj.RobotController;
import java.util.Optional;

public enum RobotKind {
  TURRET_BOT("placeholder");

  /**
   * Returns the RobotKind by matching the serial number to a known RobotKind. If the serial number
   * can't be matched, returns empty.
   */
  public static Optional<RobotKind> fromSerialNumber() {
    return switch (RobotController.getSerialNumber()) {
      case "placeholder" -> Optional.of(TURRET_BOT);
      default -> Optional.empty();
    };
  }

  public final String serialNumber;

  RobotKind(String serialNumber) {
    this.serialNumber = serialNumber;
  }
}
