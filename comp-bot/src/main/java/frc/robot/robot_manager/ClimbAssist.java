package frc.robot.robot_manager;

import com.team581.math.PoseErrorTolerance;
import com.team581.trailblazer.AutoPoint;
import com.team581.trailblazer.Trailblazer;
import com.team581.trailblazer.segments.AutoSegment;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import frc.robot.climber.ClimbLocation;

public class ClimbAssist {
  private static final Transform2d APPROACH_TRANSFORM = new Transform2d(0.5, 0.0, Rotation2d.kZero);

  private static final PoseErrorTolerance APPROACH_TOLERANCE =
      new PoseErrorTolerance(Units.inchesToMeters(5.0), Rotation2d.fromDegrees(5));

  private static final double MAX_LINEAR_VELOCITY = 2.0;
  private static final double MAX_LINEAR_ACCELERATION = 1.0;

  public static AutoSegment getClimbAssistSegment(Pose2d robot, ClimbLocation location) {
    var goalPoint = location.getEndGoalPoint(robot);
    var approachPoint =
        goalPoint.transformBy(APPROACH_TRANSFORM).withTransitionTolerance(APPROACH_TOLERANCE);

    return Trailblazer.segment(approachPoint, AutoPoint.of(goalPoint))
        .withLinearConstraints(MAX_LINEAR_VELOCITY, MAX_LINEAR_ACCELERATION)
        .forever();
  }
}
