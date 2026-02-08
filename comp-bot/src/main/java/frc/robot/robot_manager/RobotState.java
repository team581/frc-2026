package frc.robot.robot_manager;

public enum RobotState {
  IDLE,

  PREPARE_SCORE,
  SCORE,

  PREPARE_PRESET_SCORE,
  PRESET_SCORE,

  PREPARE_PRESET_FEED,
  PRESET_FEED,

  PREPARE_FEED,
  FEED,

  UNJAM,

  CLIMB_1_LINEUP_L1_AUTONOMOUS,
  CLIMB_2_RAISING_L1_AUTONOMOUS,
  CLIMB_3_HANGING_L1_AUTONOMOUS,
  CLIMB_4_RELEASE_L1_AUTONOMOUS,

  AUTOMATIC_CLIMB_1_LINEUP_L1,
  AUTOMATIC_CLIMB_2_RAISING_L1,
  AUTOMATIC_CLIMB_3_HANGING_L1,

  AUTOMATIC_CLIMB_4_RAISING_L2,
  AUTOMATIC_CLIMB_5_HANGING_L2,

  AUTOMATIC_CLIMB_6_RAISING_L3,
  AUTOMATIC_CLIMB_7_HANGING_L3,

  MANUAL_CLIMB_1_LINEUP_L1,
  MANUAL_CLIMB_2_RAISING_L1,
  MANUAL_CLIMB_3_HANGING_L1,

  MANUAL_CLIMB_4_RAISING_L2,
  MANUAL_CLIMB_5_HANGING_L2,

  MANUAL_CLIMB_6_RAISING_L3,
  MANUAL_CLIMB_7_HANGING_L3;

  public boolean isClimbing() {
    return switch (this) {
      case CLIMB_1_LINEUP_L1_AUTONOMOUS -> true;
      case CLIMB_2_RAISING_L1_AUTONOMOUS -> true;
      case CLIMB_3_HANGING_L1_AUTONOMOUS -> true;

      case AUTOMATIC_CLIMB_1_LINEUP_L1 -> true;
      case AUTOMATIC_CLIMB_2_RAISING_L1 -> true;
      case AUTOMATIC_CLIMB_3_HANGING_L1 -> true;
      case AUTOMATIC_CLIMB_4_RAISING_L2 -> true;
      case AUTOMATIC_CLIMB_5_HANGING_L2 -> true;
      case AUTOMATIC_CLIMB_6_RAISING_L3 -> true;
      case AUTOMATIC_CLIMB_7_HANGING_L3 -> true;

      case MANUAL_CLIMB_1_LINEUP_L1 -> true;
      case MANUAL_CLIMB_2_RAISING_L1 -> true;
      case MANUAL_CLIMB_3_HANGING_L1 -> true;
      case MANUAL_CLIMB_4_RAISING_L2 -> true;
      case MANUAL_CLIMB_5_HANGING_L2 -> true;
      case MANUAL_CLIMB_6_RAISING_L3 -> true;
      case MANUAL_CLIMB_7_HANGING_L3 -> true;

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
