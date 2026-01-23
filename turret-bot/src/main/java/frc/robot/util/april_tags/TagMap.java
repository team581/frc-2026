package frc.robot.util.april_tags;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.HashMap;
import java.util.Map;

/**
 * Field constants and AprilTag layout for the FRC 2026 WELDED field map. * Data Source:
 * FRC2026_WELDED.fmap Origin: Appears to be Center of Field (based on negative coordinates).
 */
public class TagMap {

  public static final double FIELD_LENGTH_METERS = 16.541;
  public static final double FIELD_WIDTH_METERS = 8.069;

  private static final Map<Integer, Pose2d> APRIL_TAGS = new HashMap<>();

  static {
    APRIL_TAGS.put(1, new Pose2d(11.87798, 7.42478, Rotation2d.k180deg));
    APRIL_TAGS.put(2, new Pose2d(11.91542, 4.63804, Rotation2d.kCCW_90deg));
    APRIL_TAGS.put(3, new Pose2d(11.31186, 4.39024, Rotation2d.k180deg));
    APRIL_TAGS.put(4, new Pose2d(11.31186, 4.03464, Rotation2d.k180deg));
    APRIL_TAGS.put(5, new Pose2d(11.91542, 3.43124, Rotation2d.kCW_90deg));
    APRIL_TAGS.put(6, new Pose2d(11.87798, 0.64450, Rotation2d.k180deg));

    APRIL_TAGS.put(7, new Pose2d(11.95288, 0.64450, Rotation2d.kZero));
    APRIL_TAGS.put(8, new Pose2d(12.27102, 3.43124, Rotation2d.kCW_90deg));
    APRIL_TAGS.put(9, new Pose2d(12.51918, 3.67904, Rotation2d.kZero));
    APRIL_TAGS.put(10, new Pose2d(12.51918, 4.03464, Rotation2d.kZero));
    APRIL_TAGS.put(11, new Pose2d(12.27102, 4.63804, Rotation2d.kCCW_90deg));
    APRIL_TAGS.put(12, new Pose2d(11.95288, 7.42478, Rotation2d.kZero));

    APRIL_TAGS.put(13, new Pose2d(16.53332, 7.40331, Rotation2d.k180deg));
    APRIL_TAGS.put(14, new Pose2d(16.53332, 6.97151, Rotation2d.k180deg));
    APRIL_TAGS.put(15, new Pose2d(16.53296, 4.32356, Rotation2d.k180deg));
    APRIL_TAGS.put(16, new Pose2d(16.53296, 3.89176, Rotation2d.k180deg));

    APRIL_TAGS.put(17, new Pose2d(4.66308, 0.64450, Rotation2d.kZero));
    APRIL_TAGS.put(18, new Pose2d(4.62562, 3.43124, Rotation2d.kCW_90deg));
    APRIL_TAGS.put(19, new Pose2d(5.22917, 3.67904, Rotation2d.kZero));
    APRIL_TAGS.put(20, new Pose2d(5.22917, 4.03464, Rotation2d.kZero));
    APRIL_TAGS.put(21, new Pose2d(4.62562, 4.63804, Rotation2d.kCCW_90deg));
    APRIL_TAGS.put(22, new Pose2d(4.66308, 7.42478, Rotation2d.kZero));

    APRIL_TAGS.put(23, new Pose2d(4.58818, 7.42478, Rotation2d.k180deg));
    APRIL_TAGS.put(24, new Pose2d(4.27002, 4.63804, Rotation2d.kCCW_90deg));
    APRIL_TAGS.put(25, new Pose2d(4.02186, 4.39024, Rotation2d.k180deg));
    APRIL_TAGS.put(26, new Pose2d(4.02186, 4.03464, Rotation2d.k180deg));
    APRIL_TAGS.put(27, new Pose2d(4.27002, 3.43124, Rotation2d.kCW_90deg));
    APRIL_TAGS.put(28, new Pose2d(4.58818, 0.64450, Rotation2d.k180deg));

    APRIL_TAGS.put(29, new Pose2d(0.00775, 0.66596, Rotation2d.kZero));
    APRIL_TAGS.put(30, new Pose2d(0.00775, 1.09776, Rotation2d.kZero));
    APRIL_TAGS.put(31, new Pose2d(0.00808, 3.74571, Rotation2d.kZero));
    APRIL_TAGS.put(32, new Pose2d(0.00808, 4.17751, Rotation2d.kZero));
  }

  /**
   * Gets the Pose2d of a specific AprilTag ID from 2026
   *
   * @param id The AprilTag ID (1-32)
   * @return The Pose2d of the tag (Translation X, Y and Rotation Z), or null if ID is invalid.
   */
  public static Pose2d getTagPose(int id) {
    return APRIL_TAGS.get(id);
  }
}
