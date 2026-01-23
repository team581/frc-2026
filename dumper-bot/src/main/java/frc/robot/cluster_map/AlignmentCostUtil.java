package frc.robot.cluster_map;

import com.team581.math.MathHelpers;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class AlignmentCostUtil {

  private static final double DRIVE_DIRECTION_SCALAR = 0.02;

  public static double getClusterAlignCost(
      Pose2d target, Pose2d robotPose, ChassisSpeeds robotVelocity) {
    var distanceCost = target.getTranslation().getDistance(robotPose.getTranslation());
    if (target.getTranslation().equals(Translation2d.kZero)
        || robotPose.getTranslation().equals(Translation2d.kZero)) {
      return distanceCost;
    }

    if (MathHelpers.getLinearVelocity(robotVelocity) == 0.0) {
      return distanceCost;
    }

    var targetRobotRelative = target.getTranslation().minus(robotPose.getTranslation());
    var targetDirection = targetRobotRelative.getAngle();

    var driveAngleCost =
        DRIVE_DIRECTION_SCALAR
            * Math.abs(
                targetDirection.minus(MathHelpers.getDriveDirection(robotVelocity)).getRadians());
    return distanceCost + driveAngleCost;
  }
}
