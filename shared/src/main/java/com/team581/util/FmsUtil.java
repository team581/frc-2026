package com.team581.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.Optional;

public class FmsUtil {
  // Match period shift times in seconds
  private static final double SHIFT1_TIMESTAMP = 33.0;
  private static final double SHIFT2_TIMESTAMP = 58.0;
  private static final double SHIFT3_TIMESTAMP = 83.0;
  private static final double SHIFT4_TIMESTAMP = 108.0;
  private static final double ENDGAME_TIMESTAMP = 133.0;

  public static final double ENDGAME_DURATION = 163.0;
  private static final double SHIFT4_TIME_DURATION = 133.0;
  private static final double SHIFT3_TIME_DURATION = 108.0;
  private static final double SHIFT2_TIME_DURATION = 83.0;
  private static final double SHIFT1_TIME_DURATION = 58.0;
  private static final double TRANSITION_DURATION = 33.0;

  private static final double AUTO_DURATION = 20.0;

  public static final double MATCH_TIME_AT_TELEOP_START = 23.0;
  public static final double MATCH_TIME_AT_AUTO_START = 0;

  public static String currentShift(double timeSinceMatchStart) {
    if (DriverStation.isDisabled() || !DriverStation.isTeleop()) {
      return "";
    }
    var maybeWonAuto = isAutoWinner();
    if (timeSinceMatchStart <= TRANSITION_DURATION) {
      if (maybeWonAuto.isEmpty()) {
        return "TRANSITION";
      }
      if (maybeWonAuto.orElseThrow()) {
        return "WON";
      }
      return "LOST";
    }
    if (timeSinceMatchStart <= SHIFT1_TIME_DURATION) {
      return "SHIFT 1";
    }
    if (timeSinceMatchStart <= SHIFT2_TIME_DURATION) {
      return "SHIFT 2";
    }
    if (timeSinceMatchStart <= SHIFT3_TIME_DURATION) {
      return "SHIFT 3";
    }
    if (timeSinceMatchStart <= SHIFT4_TIME_DURATION) {
      return "SHIFT 4";
    }
    if (timeSinceMatchStart <= ENDGAME_DURATION) {
      return "ENDGAME";
    }
    return "";
  }

  public static boolean isHubActive(double timeSinceMatchStart, boolean defaultAutoWinnerValue) {
    var isAutoWinner = isAutoWinner().orElse(defaultAutoWinnerValue);

    if (timeSinceMatchStart >= ENDGAME_TIMESTAMP) {
      return true;
    }
    if (timeSinceMatchStart >= SHIFT4_TIMESTAMP) {
      return isAutoWinner;
    }
    if (timeSinceMatchStart >= SHIFT3_TIMESTAMP) {
      return !isAutoWinner;
    }
    if (timeSinceMatchStart >= SHIFT2_TIMESTAMP) {
      return isAutoWinner;
    }
    if (timeSinceMatchStart >= SHIFT1_TIMESTAMP) {
      return !isAutoWinner;
    }
    return true;
  }

  public static boolean isRedAlliance() {
    var alliance = DriverStation.getAlliance().orElse(Alliance.Red);

    return alliance == Alliance.Red;
  }

  public static double timeUntilNextShift(
      double timeSinceMatchStart, boolean defaultAutoWinnerValue) {

    if (timeSinceMatchStart <= AUTO_DURATION) {
      return AUTO_DURATION - timeSinceMatchStart;
    }
    if (timeSinceMatchStart <= MATCH_TIME_AT_TELEOP_START) {
      return MATCH_TIME_AT_TELEOP_START - timeSinceMatchStart;
    }
    if (timeSinceMatchStart <= TRANSITION_DURATION) {
      return TRANSITION_DURATION - timeSinceMatchStart;
    }
    if (timeSinceMatchStart <= SHIFT1_TIME_DURATION) {
      return SHIFT1_TIME_DURATION - timeSinceMatchStart;
    }
    if (timeSinceMatchStart <= SHIFT2_TIME_DURATION) {
      return SHIFT2_TIME_DURATION - timeSinceMatchStart;
    }
    if (timeSinceMatchStart <= SHIFT3_TIME_DURATION) {
      return SHIFT3_TIME_DURATION - timeSinceMatchStart;
    }
    if (timeSinceMatchStart <= SHIFT4_TIME_DURATION) {
      if (isAutoWinner().orElse(defaultAutoWinnerValue)) {
        return ENDGAME_DURATION - timeSinceMatchStart;
      }
      return SHIFT4_TIME_DURATION - timeSinceMatchStart;
    }
    if (timeSinceMatchStart <= ENDGAME_DURATION) {
      return ENDGAME_DURATION - timeSinceMatchStart;
    }
    return 0.0;
  }

  private static Optional<Boolean> isAutoWinner() {
    if (RobotBase.isSimulation()) {
      return Optional.empty();
    }
    var gameData = DriverStation.getGameSpecificMessage();
    if (gameData.isEmpty()) {
      return Optional.empty();
    }
    var character = gameData.charAt(0);
    return switch (character) {
      case 'R' -> Optional.of(isRedAlliance());
      case 'B' -> Optional.of(!isRedAlliance());
      default -> Optional.empty();
    };
  }

  private FmsUtil() {}
}
