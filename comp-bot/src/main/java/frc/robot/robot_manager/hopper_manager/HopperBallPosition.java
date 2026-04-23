package frc.robot.robot_manager.hopper_manager;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;

public enum HopperBallPosition {
  // Calculated from average of CAPIN logs
  CLOSE_TO_SHOOTER(0.25),
  AT_SENSOR(0.2),
  BELOW_SENSOR(0.3);

  private final DoubleSubscriber FEEDER_TO_SHOOTER_TRAVEL_TIME;

  private HopperBallPosition(double defaultTime) {
    this.FEEDER_TO_SHOOTER_TRAVEL_TIME =
        DogLog.tunable("Hopper/FeederToShooterTravelTime/" + this.name(), defaultTime);
  }

  /**
   * Serves as lookahead time
   *
   * @return Time for the balls to travel from the feeder to the shooter
   */
  public double getTimeToShooter() {
    return this.FEEDER_TO_SHOOTER_TRAVEL_TIME.get();
  }
}
