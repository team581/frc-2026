package com.team581.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import java.util.Optional;

public class FmsUtil {
  // Match period shift times in seconds
  private static final double TRANSITION_SHIFT_TIME_STAMP = 20.0;
  private static final double SHIFT1TIME_STAMP = 30.0;
  private static final double SHIFT2TIME_STAMP = 55.0;
  private static final double SHIFT3TIME_STAMP = 80.0;
  private static final double SHIFT4TIME_STAMP = 105.0;
  private static final double END_GAME_TIME_STAMP = 130.0;

  private static final double ENDGAME_DURATION = 160.0;
  private static final double SHIFT4TIME_DURATION = 130.0;
  private static final double SHIFT3TIME_DURATION = 105.0;
  private static final double SHIFT2TIME_DURATION = 80.0;
  private static final double SHIFT1TIME_DURATION = 55.0;
  private static final double TRANSITION_DURATION = 30.0;

  public static final double MATCH_TIME_AT_TELEOP_START = 20.0;
  public static Timer activeTimer = new Timer();

  public static boolean isHubActive(double timeSinceMatchStart) {
    var maybeIsAutoWinner = isAutoWinner();

    if (maybeIsAutoWinner.isEmpty()) {
      return true;
    }

    return isHubActive(timeSinceMatchStart, maybeIsAutoWinner.orElseThrow());
  }

  public static boolean isRedAlliance() {
    var alliance = DriverStation.getAlliance().orElse(Alliance.Red);

    return alliance == Alliance.Red;
  }

  public static double timeUntilInactive(double timeSinceMatchStart, boolean isHubActive) {
    if (isHubActive) {
      if (timeSinceMatchStart >= END_GAME_TIME_STAMP) {
        return ENDGAME_DURATION - timeSinceMatchStart;
      }
      if (timeSinceMatchStart >= SHIFT4TIME_STAMP) {
        return SHIFT4TIME_DURATION - timeSinceMatchStart;
      }
      if (timeSinceMatchStart >= SHIFT3TIME_STAMP) {
        return SHIFT3TIME_DURATION - timeSinceMatchStart;
      }
      if (timeSinceMatchStart >= SHIFT2TIME_STAMP) {
        return SHIFT2TIME_DURATION - timeSinceMatchStart;
      }
      if (timeSinceMatchStart >= SHIFT1TIME_STAMP) {
        return SHIFT1TIME_DURATION - timeSinceMatchStart;
      }
      if (timeSinceMatchStart >= TRANSITION_SHIFT_TIME_STAMP) {
        return TRANSITION_DURATION - timeSinceMatchStart;
      }
    }
    return 0;
  }

  public static double timeUntilNextShift(double timeSinceMatchStart) {
    if (DriverStation.isDisabled() || !DriverStation.isTeleop()) {
      return 0.0;
    }
    double timeUntilSwitch = 0.0;

    if (timeSinceMatchStart <= ENDGAME_DURATION) {
      timeUntilSwitch = ENDGAME_DURATION - timeSinceMatchStart;
    }
    if (timeSinceMatchStart <= SHIFT4TIME_DURATION) {
      timeUntilSwitch = SHIFT4TIME_DURATION - timeSinceMatchStart;
    }
    if (timeSinceMatchStart <= SHIFT3TIME_DURATION) {
      timeUntilSwitch = SHIFT3TIME_DURATION - timeSinceMatchStart;
    }
    if (timeSinceMatchStart <= SHIFT2TIME_DURATION) {
      timeUntilSwitch = SHIFT2TIME_DURATION - timeSinceMatchStart;
    }
    if (timeSinceMatchStart <= SHIFT1TIME_DURATION) {
      timeUntilSwitch = SHIFT1TIME_DURATION - timeSinceMatchStart;
    }
    if (timeSinceMatchStart <= TRANSITION_DURATION) {
      timeUntilSwitch = TRANSITION_DURATION - timeSinceMatchStart;
    }
    return timeUntilSwitch;
  }

  public static String currentShift(double timeSinceMatchStart) {
    if (DriverStation.isDisabled() || !DriverStation.isTeleop()) {
      return "";
    }
    var shift = "";

    if (timeSinceMatchStart <= ENDGAME_DURATION) {
      shift= "ENDGAME";
    }
    if (timeSinceMatchStart <= SHIFT4TIME_DURATION) {
      shift = " SHIFT 4";
    }
    if (timeSinceMatchStart <= SHIFT3TIME_DURATION) {
      shift = " SHIFT 3";
    }
    if (timeSinceMatchStart <= SHIFT2TIME_DURATION) {
      shift = " SHIFT 2";
    }
    if (timeSinceMatchStart <= SHIFT1TIME_DURATION) {
      shift = " SHIFT 1";
    }
    if (timeSinceMatchStart <= TRANSITION_DURATION) {
      shift = "TRANSITION";
    }
    return shift;
  }

  static Optional<Boolean> isAutoWinner() {
    if (RobotBase.isSimulation()) {
      return Optional.of(true);
    }
    var gameData = DriverStation.getGameSpecificMessage();
    if (gameData.isEmpty()) {
      return Optional.of(true);
    }
    var character = gameData.charAt(0);
    return switch (character) {
      case 'R' -> Optional.of(isRedAlliance());
      case 'B' -> Optional.of(!isRedAlliance());
      default -> Optional.empty();
    };
  }

  static boolean isHubActive(double timeSinceMatchStart, boolean isAutoWinner) {
    if (timeSinceMatchStart >= END_GAME_TIME_STAMP) {
      return true;
    }
    if (timeSinceMatchStart >= SHIFT4TIME_STAMP) {
      return isAutoWinner;
    }
    if (timeSinceMatchStart >= SHIFT3TIME_STAMP) {
      return !isAutoWinner;
    }
    if (timeSinceMatchStart >= SHIFT2TIME_STAMP) {
      return isAutoWinner;
    }
    if (timeSinceMatchStart >= SHIFT1TIME_STAMP) {
      return !isAutoWinner;
    }
    if (timeSinceMatchStart >= TRANSITION_SHIFT_TIME_STAMP) {
      return true;
    }
    return true;
  }

  private FmsUtil() {}
}
