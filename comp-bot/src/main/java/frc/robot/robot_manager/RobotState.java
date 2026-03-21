package frc.robot.robot_manager;

public enum RobotState {
  IDLE,
  // for tuning shooter purposes only
  PREPARE_FORCE_SCORE,
  FORCE_SCORE,

  PREPARE_SCORE,
  SCORE,
  STOP_SHOOTING_SCORE,

  PREPARE_PRESET_SCORE,
  PRESET_SCORE,
  STOP_SHOOTING_PRESET_SCORE,

  PREPARE_PRESET_FEED,
  PRESET_FEED,
  STOP_SHOOTING_PRESET_FEED,

  PREPARE_FEED,
  FEED,
  STOP_SHOOTING_FEED,

  UNJAM;

  public boolean isClimbing() {
    return switch (this) {
      default -> false;
    };
  }

  public boolean isFeeding() {
    return switch (this) {
      case PREPARE_PRESET_FEED -> true;
      case PRESET_FEED -> true;
      case PREPARE_FEED -> true;
      case FEED -> true;
      default -> false;
    };
  }

  public boolean isScoring() {
    return switch (this) {
      case PREPARE_PRESET_SCORE -> true;
      case PRESET_SCORE -> true;
      case PREPARE_SCORE -> true;
      case SCORE -> true;
      default -> false;
    };
  }
}
