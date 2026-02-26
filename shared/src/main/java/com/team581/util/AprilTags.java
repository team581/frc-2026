package com.team581.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import org.jspecify.annotations.Nullable;

public class AprilTags {
  public static final AprilTagFieldLayout FIELD_LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  public static final Pose3d TAG_1 = FIELD_LAYOUT.getTagPose(1).orElseThrow();
  public static final Pose3d TAG_2 = FIELD_LAYOUT.getTagPose(2).orElseThrow();
  public static final Pose3d TAG_3 = FIELD_LAYOUT.getTagPose(3).orElseThrow();
  public static final Pose3d TAG_4 = FIELD_LAYOUT.getTagPose(4).orElseThrow();
  public static final Pose3d TAG_5 = FIELD_LAYOUT.getTagPose(5).orElseThrow();
  public static final Pose3d TAG_6 = FIELD_LAYOUT.getTagPose(6).orElseThrow();
  public static final Pose3d TAG_7 = FIELD_LAYOUT.getTagPose(7).orElseThrow();
  public static final Pose3d TAG_8 = FIELD_LAYOUT.getTagPose(8).orElseThrow();
  public static final Pose3d TAG_9 = FIELD_LAYOUT.getTagPose(9).orElseThrow();
  public static final Pose3d TAG_10 = FIELD_LAYOUT.getTagPose(10).orElseThrow();
  public static final Pose3d TAG_11 = FIELD_LAYOUT.getTagPose(11).orElseThrow();
  public static final Pose3d TAG_12 = FIELD_LAYOUT.getTagPose(12).orElseThrow();
  public static final Pose3d TAG_13 = FIELD_LAYOUT.getTagPose(13).orElseThrow();
  public static final Pose3d TAG_14 = FIELD_LAYOUT.getTagPose(14).orElseThrow();
  public static final Pose3d TAG_15 = FIELD_LAYOUT.getTagPose(15).orElseThrow();
  public static final Pose3d TAG_16 = FIELD_LAYOUT.getTagPose(16).orElseThrow();
  public static final Pose3d TAG_17 = FIELD_LAYOUT.getTagPose(17).orElseThrow();
  public static final Pose3d TAG_18 = FIELD_LAYOUT.getTagPose(18).orElseThrow();
  public static final Pose3d TAG_19 = FIELD_LAYOUT.getTagPose(19).orElseThrow();
  public static final Pose3d TAG_20 = FIELD_LAYOUT.getTagPose(20).orElseThrow();
  public static final Pose3d TAG_21 = FIELD_LAYOUT.getTagPose(21).orElseThrow();
  public static final Pose3d TAG_22 = FIELD_LAYOUT.getTagPose(22).orElseThrow();
  public static final Pose3d TAG_23 = FIELD_LAYOUT.getTagPose(23).orElseThrow();
  public static final Pose3d TAG_24 = FIELD_LAYOUT.getTagPose(24).orElseThrow();
  public static final Pose3d TAG_25 = FIELD_LAYOUT.getTagPose(25).orElseThrow();
  public static final Pose3d TAG_26 = FIELD_LAYOUT.getTagPose(26).orElseThrow();
  public static final Pose3d TAG_27 = FIELD_LAYOUT.getTagPose(27).orElseThrow();
  public static final Pose3d TAG_28 = FIELD_LAYOUT.getTagPose(28).orElseThrow();
  public static final Pose3d TAG_29 = FIELD_LAYOUT.getTagPose(29).orElseThrow();
  public static final Pose3d TAG_30 = FIELD_LAYOUT.getTagPose(30).orElseThrow();
  public static final Pose3d TAG_31 = FIELD_LAYOUT.getTagPose(31).orElseThrow();
  public static final Pose3d TAG_32 = FIELD_LAYOUT.getTagPose(32).orElseThrow();

  public static Pose2d getClimbTagPose() {
    return FmsUtil.isRedAlliance() ? TAG_15.toPose2d() : TAG_31.toPose2d();
  }

  /**
   * Gets the Pose2d of a specific AprilTag ID.
   *
   * @param id The AprilTag ID
   * @return The Pose2d of the tag (Translation X, Y and Rotation Z), or null if ID is invalid.
   */
  public static @Nullable Pose2d getTagPose(int id) {
    return switch (id) {
      case 1 -> TAG_1.toPose2d();
      case 2 -> TAG_2.toPose2d();
      case 3 -> TAG_3.toPose2d();
      case 4 -> TAG_4.toPose2d();
      case 5 -> TAG_5.toPose2d();
      case 6 -> TAG_6.toPose2d();
      case 7 -> TAG_7.toPose2d();
      case 8 -> TAG_8.toPose2d();
      case 9 -> TAG_9.toPose2d();
      case 10 -> TAG_10.toPose2d();
      case 11 -> TAG_11.toPose2d();
      case 12 -> TAG_12.toPose2d();
      case 13 -> TAG_13.toPose2d();
      case 14 -> TAG_14.toPose2d();
      case 15 -> TAG_15.toPose2d();
      case 16 -> TAG_16.toPose2d();
      case 17 -> TAG_17.toPose2d();
      case 18 -> TAG_18.toPose2d();
      case 19 -> TAG_19.toPose2d();
      case 20 -> TAG_20.toPose2d();
      case 21 -> TAG_21.toPose2d();
      case 22 -> TAG_22.toPose2d();
      case 23 -> TAG_23.toPose2d();
      case 24 -> TAG_24.toPose2d();
      case 25 -> TAG_25.toPose2d();
      case 26 -> TAG_26.toPose2d();
      case 27 -> TAG_27.toPose2d();
      case 28 -> TAG_28.toPose2d();
      case 29 -> TAG_29.toPose2d();
      case 30 -> TAG_30.toPose2d();
      case 31 -> TAG_31.toPose2d();
      case 32 -> TAG_32.toPose2d();
      default -> null;
    };
  }

  private AprilTags() {}
}
