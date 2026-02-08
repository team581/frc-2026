package com.team581.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.Optional;

public class FmsUtil {
  // Match period shift times in seconds
  private static final double TRANSITION_SHIFT_TIME_STAMP = 20.0;
  private static final double SHIFT1TIME_STAMP = 30.0;
  private static final double SHIFT2TIME_STAMP = 55.0;
  private static final double SHIFT3TIME_STAMP = 80.0;
  private static final double SHIFT4TIME_STAMP = 105.0;
  private static final double END_GAME_TIME_STAMP = 130.0;

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

  static Optional<Boolean> isAutoWinner() {
    if (RobotBase.isSimulation()) {
      return Optional.of(true);
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
