package frc.robot.autos;

import com.team581.autos.Point;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public enum Points {
  START_R1_AND_B1(Point.ofRed(new Pose2d(12.925, 2.023, Rotation2d.k180deg))),
  START_R2_AND_B2(Point.ofRed(new Pose2d(12.954, 2.485, Rotation2d.k180deg))),
  START_R3_AND_B3(Point.ofRed(new Pose2d(12.939, 2.97, Rotation2d.k180deg))),
  START_R4_AND_B4(Point.ofRed(new Pose2d(12.939, 5.074, Rotation2d.k180deg))),
  START_R5_AND_B5(Point.ofRed(new Pose2d(12.928, 5.557, Rotation2d.k180deg))),
  START_R6_AND_B6(Point.ofRed(new Pose2d(12.959, 6.02, Rotation2d.k180deg))),

  INTAKE_MIDLINE_RIGHT(Point.ofRed(new Pose2d(9.64, 5.938, Rotation2d.k180deg))),
  INTAKE_MIDLINE_LEFT(Point.ofRed(new Pose2d(9.64, 2.154, Rotation2d.k180deg))),
  INTAKE_DEPOT(Point.ofRed(new Pose2d(15.469, 2.118, Rotation2d.kZero))),

  HUB_CENTER(Point.ofRed(new Pose2d(14.505, 4.0, Rotation2d.k180deg))),
  HUB_RIGHT(Point.ofRed(new Pose2d(14.303, 4.602, Rotation2d.fromDegrees(-130.0)))),
  HUB_LEFT(Point.ofRed(new Pose2d(14.56, 3.034, Rotation2d.fromDegrees(145.1))));

  public final Point point;

  Points(Point point) {
    this.point = point;
  }

  public Pose2d getPose() {
    return point.getPose();
  }
}
