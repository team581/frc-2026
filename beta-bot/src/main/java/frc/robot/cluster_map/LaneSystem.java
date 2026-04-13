package frc.robot.cluster_map;

import com.team581.util.FieldUtil;
import com.team581.util.FmsUtil;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;

// Make sure to import your FieldUtil and Lane enum here!

public class LaneSystem {
  private final double minX, maxX;
  private final double zoneMinY, zoneMaxY;
  private final int numLanes;

  public LaneSystem(double minX, double maxX, double zoneMinY, double zoneMaxY, int numLanes) {
    this.minX = Math.min(minX, maxX);
    this.maxX = Math.max(minX, maxX);

    this.zoneMinY = zoneMinY;
    this.zoneMaxY = zoneMaxY;
    this.numLanes = numLanes;
  }

  public Lane getLane(Pose2d targetPose, Pose2d robotPose) {
    var isRed = FmsUtil.isRedAlliance();

    var activeTarget = targetPose;
    var activeRobot = robotPose;

    if (!isRed) {
      activeTarget = FieldUtil.pathflip(targetPose);
      activeRobot = FieldUtil.pathflip(robotPose);
    }

    double x = activeTarget.getX();
    DogLog.log("ActiveX", x);
    double y = activeTarget.getY();
    double robotY = activeRobot.getY();

    if (x >= minX && x <= maxX) {

      boolean robotInTopHalf = robotY >= FieldUtil.FIELD_WIDTH_Y / 2.0;

      if (y >= zoneMaxY && robotInTopHalf) {
        return Lane.TRENCH;
      } else if (y <= zoneMinY && !robotInTopHalf) {
        return Lane.TRENCH;
      }

      if (y > zoneMinY && y < zoneMaxY) {
        double laneWidth = (maxX - minX) / numLanes;

        int laneIndex = (int) ((maxX - x) / laneWidth);

        laneIndex = MathUtil.clamp(laneIndex, 0, numLanes - 1);

        return switch (laneIndex) {
          case 0 -> Lane.LANE_0;
          case 1 -> Lane.LANE_1;
          case 2 -> Lane.LANE_2;
          case 3 -> Lane.LANE_3;
          case 4 -> Lane.LANE_4;
          default -> Lane.NONE;
        };
      }
    }
    return Lane.NONE;
  }
}
