package frc.robot.climber;

import com.team581.autos.Point;
import edu.wpi.first.math.geometry.Pose2d;

public enum ClimbLocation {
  /** The default climbing location, whatever is closest to the robot. */
  LEFT,
  RIGHT,
  CLOSEST;

  public static ClimbLocation getNearest(Pose2d robot) {
    var yDistanceToLeft = Math.abs(robot.getY() - ClimberConfig.CLIMB_LEFT_LOCATION.getY());
    var yDistanceToRight = Math.abs(robot.getY() - ClimberConfig.CLIMB_RIGHT_LOCATION.getY());
    var closestClimbLocation = Math.min(yDistanceToLeft, yDistanceToRight);
    if (closestClimbLocation == yDistanceToLeft) {
      return ClimbLocation.LEFT;
    }

    return ClimbLocation.RIGHT;
  }

  public Point getEndGoalPoint(Pose2d robot) {
    return switch (this) {
      case LEFT -> ClimberConfig.CLIMB_LEFT_LOCATION;
      case RIGHT -> ClimberConfig.CLIMB_RIGHT_LOCATION;
      case CLOSEST ->
          getNearest(robot) == ClimbLocation.LEFT
              ? ClimberConfig.CLIMB_LEFT_LOCATION
              : ClimberConfig.CLIMB_RIGHT_LOCATION;
    };
  }
}
