package frc.robot.autos;

import com.team581.autos.Point;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public enum Points {
  NON_PROCESSOR_SIDE_START_ANGLED(
      new Point(
          new Pose2d(10.546, 1.782, Rotation2d.fromDegrees(60.0)),
          new Pose2d(7.042, 5.976, Rotation2d.fromDegrees(240.0)))),

  PROCESSOR_SIDE_START_ANGLED(
      new Point(
          new Pose2d(10.546, 6.218, Rotation2d.fromDegrees(300.0)),
          new Pose2d(7.042, 2.024, Rotation2d.fromDegrees(120.0))));

  public final Point point;

  Points(Point point) {
    this.point = point;
  }

  public Pose2d getPose() {
    return point.getPose();
  }
}
