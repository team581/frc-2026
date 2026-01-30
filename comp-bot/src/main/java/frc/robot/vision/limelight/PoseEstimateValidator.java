package frc.robot.vision.limelight;

import com.team581.vision.limelight.LimelightHelpers.PoseEstimate;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;

public class PoseEstimateValidator {

  public static boolean shouldTrust(
      PoseEstimate poseEstimate, double angularVelocity, String name) {
    if (poseEstimate == null) {
      return false;
    }

    if (Math.abs(angularVelocity) > 360) {
      return false;
    }
    if (poseEstimate.tagCount == 0) {
      DogLog.log("Vision/" + name + "/Tags/RawLimelightPose", Pose2d.kZero);
      return false;
    }
    if (poseEstimate.rawFiducials.length == 1) {
      double ambiguity = poseEstimate.rawFiducials[0].ambiguity;
      if (ambiguity >= 0.7) {
        DogLog.timestamp("Vision/" + name + "/Tags/AmbiguityFilter");
        return false;
      }
    }

    var mtPose = poseEstimate.pose;

    // This prevents pose estimator from having crazy poses if the Limelight loses power
    if (mtPose.getX() == 0.0 && mtPose.getY() == 0.0) {
      DogLog.log("Vision/" + name + "/Tags/RawLimelightPose", Pose2d.kZero);
      return false;
    }

    return true;
  }
}
