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

  private static final double ENDGAME_DURATION = 155.0;
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

    if (timeSinceMatchStart <= END_GAME_TIME_STAMP) {
      timeUntilSwitch = END_GAME_TIME_STAMP - timeSinceMatchStart;
    }
    if (timeSinceMatchStart <= SHIFT4TIME_STAMP) {
      timeUntilSwitch = SHIFT4TIME_STAMP - timeSinceMatchStart;
    }
    if (timeSinceMatchStart <= SHIFT3TIME_STAMP) {
      timeUntilSwitch = SHIFT3TIME_STAMP - timeSinceMatchStart;
    }
    if (timeSinceMatchStart <= SHIFT2TIME_STAMP) {
      timeUntilSwitch = SHIFT2TIME_STAMP - timeSinceMatchStart;
    }
    if (timeSinceMatchStart <= SHIFT1TIME_STAMP) {
      timeUntilSwitch = SHIFT1TIME_STAMP - timeSinceMatchStart;
    }
    if (timeSinceMatchStart <= TRANSITION_SHIFT_TIME_STAMP) {
      timeUntilSwitch = TRANSITION_SHIFT_TIME_STAMP - timeSinceMatchStart;
    }
    return timeUntilSwitch;
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
