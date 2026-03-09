package com.team581.util;

import com.google.common.collect.ImmutableList;
import com.team581.autos.Point;
import com.team581.math.MathHelpers;
import com.team581.math.ObstructionCalculator;
import com.team581.math.Triangle2d;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.List;
import java.util.Optional;

public class FieldUtil {

  public static final double FIELD_LENGTH_X = AprilTags.FIELD_LAYOUT.getFieldLength();
  public static final double FIELD_WIDTH_Y = AprilTags.FIELD_LAYOUT.getFieldWidth();
  public static final Rectangle2d FIELD_BOUNDS =
      new Rectangle2d(Translation2d.kZero, new Translation2d(FIELD_LENGTH_X, FIELD_WIDTH_Y));

  public static final Point HUB_POSE =
      Point.ofRed(new Pose2d(11.915394, 4.034663, Rotation2d.kZero));
  public static final double HUB_RADIUS_METERS = Units.inchesToMeters(48.106087 / 2);
  public static final Point FEED_LEFT_POSE = Point.ofRed(new Pose2d(15.75, 1.5, Rotation2d.kZero));
  public static final Point FEED_RIGHT_POSE =
      Point.ofRed(new Pose2d(15.75, FIELD_WIDTH_Y - 1.5, Rotation2d.kZero));

  private static final double BLUE_STARTING_LINE_X = Units.inchesToMeters(156.61);
  private static final double RED_STARTING_LINE_X = FIELD_LENGTH_X - BLUE_STARTING_LINE_X;

  // OBSTACLE_X referring to the trenches and bumps x coordinate of the corresponding alliance
  private static final double RED_OBSTACLE_X =
      MathHelpers.average(AprilTags.TAG_7.getX(), AprilTags.TAG_6.getX());
  private static final double BLUE_OBSTACLE_X =
      MathHelpers.average(AprilTags.TAG_17.getX(), AprilTags.TAG_28.getX());

  // Trench/assist zone calculations
  private static final double TRENCH_LENGTH_X = Units.inchesToMeters(47.0);
  private static final double TRENCH_LENGTH_Y = Units.inchesToMeters(48.94);

  private static final double TRENCH_ZONE_LENGTH_X = TRENCH_LENGTH_X + Units.inchesToMeters(27.0);
  private static final double TRENCH_ZONE_LENGTH_Y = Units.inchesToMeters(68.0);
  private static final double TRENCH_ASSIST_ZONE_LENGTH_X = TRENCH_LENGTH_X * 5;
  private static final double TRENCH_ASSIST_ZONE_LENGTH_Y = Units.inchesToMeters(85.0);

  public static final Pose2d BLUE_OUTPOST_TRENCH_CENTER =
      new Pose2d(BLUE_OBSTACLE_X, AprilTags.TAG_7.getY(), Rotation2d.kZero);
  private static final Pose2d BLUE_DEPOT_TRENCH_CENTER =
      new Pose2d(BLUE_OBSTACLE_X, AprilTags.TAG_12.getY(), Rotation2d.kZero);
  private static final Pose2d RED_DEPOT_TRENCH_CENTER =
      new Pose2d(RED_OBSTACLE_X, AprilTags.TAG_17.getY(), Rotation2d.kZero);
  public static final Pose2d RED_OUTPOST_TRENCH_CENTER =
      new Pose2d(RED_OBSTACLE_X, AprilTags.TAG_22.getY(), Rotation2d.kZero);

  private static final Rectangle2d BLUE_OUTPOST_TRENCH_ZONE =
      new Rectangle2d(BLUE_OUTPOST_TRENCH_CENTER, TRENCH_ZONE_LENGTH_X, TRENCH_ZONE_LENGTH_Y);
  private static final Rectangle2d BLUE_DEPOT_TRENCH_ZONE =
      new Rectangle2d(BLUE_DEPOT_TRENCH_CENTER, TRENCH_ZONE_LENGTH_X, TRENCH_ZONE_LENGTH_Y);
  private static final Rectangle2d RED_DEPOT_TRENCH_ZONE =
      new Rectangle2d(RED_DEPOT_TRENCH_CENTER, TRENCH_ZONE_LENGTH_X, TRENCH_ZONE_LENGTH_Y);
  private static final Rectangle2d RED_OUTPOST_TRENCH_ZONE =
      new Rectangle2d(RED_OUTPOST_TRENCH_CENTER, TRENCH_ZONE_LENGTH_X, TRENCH_ZONE_LENGTH_Y);

  private static final Rectangle2d BLUE_OUTPOST_TRENCH_ASSIST_ZONE =
      new Rectangle2d(
          BLUE_OUTPOST_TRENCH_CENTER, TRENCH_ASSIST_ZONE_LENGTH_X, TRENCH_ASSIST_ZONE_LENGTH_Y);
  private static final Rectangle2d BLUE_DEPOT_TRENCH_ASSIST_ZONE =
      new Rectangle2d(
          BLUE_DEPOT_TRENCH_CENTER, TRENCH_ASSIST_ZONE_LENGTH_X, TRENCH_ASSIST_ZONE_LENGTH_Y);
  private static final Rectangle2d RED_DEPOT_TRENCH_ASSIST_ZONE =
      new Rectangle2d(
          RED_DEPOT_TRENCH_CENTER, TRENCH_ASSIST_ZONE_LENGTH_X, TRENCH_ASSIST_ZONE_LENGTH_Y);
  private static final Rectangle2d RED_OUTPOST_TRENCH_ASSIST_ZONE =
      new Rectangle2d(
          RED_OUTPOST_TRENCH_CENTER, TRENCH_ASSIST_ZONE_LENGTH_X, TRENCH_ASSIST_ZONE_LENGTH_Y);

  private static final List<Rectangle2d> TRENCH_ZONES =
      ImmutableList.of(
          BLUE_OUTPOST_TRENCH_ZONE,
          BLUE_DEPOT_TRENCH_ZONE,
          RED_DEPOT_TRENCH_ZONE,
          RED_OUTPOST_TRENCH_ZONE);

  private static final List<Rectangle2d> TRENCH_ASSIST_ZONES =
      ImmutableList.of(
          BLUE_OUTPOST_TRENCH_ASSIST_ZONE,
          BLUE_DEPOT_TRENCH_ASSIST_ZONE,
          RED_DEPOT_TRENCH_ASSIST_ZONE,
          RED_OUTPOST_TRENCH_ASSIST_ZONE);

  // Midpoints of the entryways of trenches to check if we are driving toward it
  private static final Translation2d BLUE_OUTPOST_ALLIANCE_ZONE_TRENCH_MIDPOINT =
      new Translation2d(BLUE_OBSTACLE_X - TRENCH_LENGTH_X / 2.0, AprilTags.TAG_7.getY());
  private static final Translation2d BLUE_DEPOT_ALLIANCE_ZONE_TRENCH_MIDPOINT =
      new Translation2d(BLUE_OBSTACLE_X - TRENCH_LENGTH_X / 2.0, AprilTags.TAG_12.getY());
  private static final Translation2d RED_DEPOT_ALLIANCE_ZONE_TRENCH_MIDPOINT =
      new Translation2d(RED_OBSTACLE_X + TRENCH_LENGTH_X / 2.0, AprilTags.TAG_17.getY());
  private static final Translation2d RED_OUTPOST_ALLIANCE_ZONE_TRENCH_MIDPOINT =
      new Translation2d(RED_OBSTACLE_X + TRENCH_LENGTH_X / 2.0, AprilTags.TAG_22.getY());

  private static final List<Translation2d> ALLIANCE_ZONE_TRENCH_MIDPOINTS =
      ImmutableList.of(
          BLUE_OUTPOST_ALLIANCE_ZONE_TRENCH_MIDPOINT, BLUE_DEPOT_ALLIANCE_ZONE_TRENCH_MIDPOINT,
          RED_DEPOT_ALLIANCE_ZONE_TRENCH_MIDPOINT, RED_OUTPOST_ALLIANCE_ZONE_TRENCH_MIDPOINT);

  private static final Translation2d BLUE_OUTPOST_NEUTRAL_ZONE_TRENCH_MIDPOINT =
      new Translation2d(BLUE_OBSTACLE_X + TRENCH_LENGTH_X / 2.0, AprilTags.TAG_7.getY());
  private static final Translation2d BLUE_DEPOT_NEUTRAL_ZONE_TRENCH_MIDPOINT =
      new Translation2d(BLUE_OBSTACLE_X + TRENCH_LENGTH_X / 2.0, AprilTags.TAG_12.getY());
  private static final Translation2d RED_DEPOT_NEUTRAL_ZONE_TRENCH_MIDPOINT =
      new Translation2d(RED_OBSTACLE_X - TRENCH_LENGTH_X / 2.0, AprilTags.TAG_17.getY());
  private static final Translation2d RED_OUTPOST_NEUTRAL_ZONE_TRENCH_MIDPOINT =
      new Translation2d(RED_OBSTACLE_X - TRENCH_LENGTH_X / 2.0, AprilTags.TAG_22.getY());

  private static final List<Translation2d> NEUTRAL_ZONE_TRENCH_MIDPOINTS =
      ImmutableList.of(
          BLUE_OUTPOST_NEUTRAL_ZONE_TRENCH_MIDPOINT, BLUE_DEPOT_NEUTRAL_ZONE_TRENCH_MIDPOINT,
          RED_DEPOT_NEUTRAL_ZONE_TRENCH_MIDPOINT, RED_OUTPOST_NEUTRAL_ZONE_TRENCH_MIDPOINT);

  // Bump/assist zone calculations
  private static final double BUMP_LENGTH_X = Units.inchesToMeters(44.4);
  private static final double BUMP_LENGTH_Y = Units.inchesToMeters(73.0);
  // Trench bump barrier is the little barrier that separates them
  private static final double TRENCH_BUMP_BARRIER_LENGTH_Y = Units.inchesToMeters(12.0);
  private static final double BUMP_CENTER_Y_DISTANCE_FROM_WALL =
      TRENCH_LENGTH_Y + TRENCH_BUMP_BARRIER_LENGTH_Y + (BUMP_LENGTH_Y / 2.0);
  private static final double BUMP_ASSIST_ZONE_LENGTH_X = BUMP_LENGTH_X * 3.0;
  private static final double BUMP_ASSIST_ZONE_LENGTH_Y = BUMP_LENGTH_Y;

  private static final Pose2d BLUE_OUTPOST_BUMP_CENTER =
      new Pose2d(BLUE_OBSTACLE_X, BUMP_CENTER_Y_DISTANCE_FROM_WALL, Rotation2d.kZero);
  private static final Pose2d BLUE_DEPOT_BUMP_CENTER =
      new Pose2d(
          BLUE_OBSTACLE_X, FIELD_WIDTH_Y - BUMP_CENTER_Y_DISTANCE_FROM_WALL, Rotation2d.kZero);
  public static final Pose2d RED_OUTPOST_BUMP_CENTER =
      new Pose2d(
          RED_OBSTACLE_X, FIELD_WIDTH_Y - BUMP_CENTER_Y_DISTANCE_FROM_WALL, Rotation2d.kZero);
  private static final Pose2d RED_DEPOT_BUMP_CENTER =
      new Pose2d(RED_OBSTACLE_X, BUMP_CENTER_Y_DISTANCE_FROM_WALL, Rotation2d.kZero);

  private static final Rectangle2d BLUE_OUTPOST_BUMP_ASSIST_ZONE =
      new Rectangle2d(
          BLUE_OUTPOST_BUMP_CENTER, BUMP_ASSIST_ZONE_LENGTH_X, BUMP_ASSIST_ZONE_LENGTH_Y);
  private static final Rectangle2d BLUE_DEPOT_BUMP_ASSIST_ZONE =
      new Rectangle2d(BLUE_DEPOT_BUMP_CENTER, BUMP_ASSIST_ZONE_LENGTH_X, BUMP_ASSIST_ZONE_LENGTH_Y);
  public static final Rectangle2d RED_OUTPOST_BUMP_ASSIST_ZONE =
      new Rectangle2d(
          RED_OUTPOST_BUMP_CENTER, BUMP_ASSIST_ZONE_LENGTH_X, BUMP_ASSIST_ZONE_LENGTH_Y);
  private static final Rectangle2d RED_DEPOT_BUMP_ASSIST_ZONE =
      new Rectangle2d(RED_DEPOT_BUMP_CENTER, BUMP_ASSIST_ZONE_LENGTH_X, BUMP_ASSIST_ZONE_LENGTH_Y);

  private static final List<Rectangle2d> BUMP_ASSIST_ZONES =
      ImmutableList.of(
          BLUE_OUTPOST_BUMP_ASSIST_ZONE,
          BLUE_DEPOT_BUMP_ASSIST_ZONE,
          RED_OUTPOST_BUMP_ASSIST_ZONE,
          RED_DEPOT_BUMP_ASSIST_ZONE);

  // Points centered on the bump x to check if we are driving toward it
  private static final Translation2d BLUE_OUTPOST_HUB_SIDE_BUMP_POINT =
      new Translation2d(
          BLUE_OBSTACLE_X, BLUE_OUTPOST_BUMP_CENTER.getY() + BUMP_ASSIST_ZONE_LENGTH_Y / 6.0);
  private static final Translation2d BLUE_DEPOT_HUB_SIDE_BUMP_POINT =
      new Translation2d(
          BLUE_OBSTACLE_X, BLUE_DEPOT_BUMP_CENTER.getY() - BUMP_ASSIST_ZONE_LENGTH_Y / 6.0);
  private static final Translation2d RED_DEPOT_HUB_SIDE_BUMP_POINT =
      new Translation2d(
          RED_OBSTACLE_X, RED_DEPOT_BUMP_CENTER.getY() + BUMP_ASSIST_ZONE_LENGTH_Y / 6.0);
  private static final Translation2d RED_OUTPOST_HUB_SIDE_BUMP_POINT =
      new Translation2d(
          RED_OBSTACLE_X, RED_OUTPOST_BUMP_CENTER.getY() - BUMP_ASSIST_ZONE_LENGTH_Y / 6.0);

  private static final List<Translation2d> HUB_SIDE_BUMP_POINTS =
      ImmutableList.of(
          BLUE_OUTPOST_HUB_SIDE_BUMP_POINT,
          BLUE_DEPOT_HUB_SIDE_BUMP_POINT,
          RED_DEPOT_HUB_SIDE_BUMP_POINT,
          RED_OUTPOST_HUB_SIDE_BUMP_POINT);

  private static final Translation2d BLUE_OUTPOST_TRENCH_SIDE_BUMP_POINT =
      new Translation2d(
          BLUE_OBSTACLE_X, BLUE_OUTPOST_BUMP_CENTER.getY() - BUMP_ASSIST_ZONE_LENGTH_Y / 6.0);
  private static final Translation2d BLUE_DEPOT_TRENCH_SIDE_BUMP_POINT =
      new Translation2d(
          BLUE_OBSTACLE_X, BLUE_DEPOT_BUMP_CENTER.getY() + BUMP_ASSIST_ZONE_LENGTH_Y / 6.0);
  private static final Translation2d RED_DEPOT_TRENCH_SIDE_BUMP_POINT =
      new Translation2d(
          RED_OBSTACLE_X, RED_DEPOT_BUMP_CENTER.getY() - BUMP_ASSIST_ZONE_LENGTH_Y / 6.0);
  private static final Translation2d RED_OUTPOST_TRENCH_SIDE_BUMP_POINT =
      new Translation2d(
          RED_OBSTACLE_X, RED_OUTPOST_BUMP_CENTER.getY() + BUMP_ASSIST_ZONE_LENGTH_Y / 6.0);

  private static final List<Translation2d> TRENCH_SIDE_BUMP_POINTS =
      ImmutableList.of(
          BLUE_OUTPOST_TRENCH_SIDE_BUMP_POINT,
          BLUE_DEPOT_TRENCH_SIDE_BUMP_POINT,
          RED_DEPOT_TRENCH_SIDE_BUMP_POINT,
          RED_OUTPOST_TRENCH_SIDE_BUMP_POINT);

  // Wall snap zone calculations
  private static final double WALL_SNAP_CORNER_ZONE_LENGTH = Units.inchesToMeters(54.0);
  private static final double NO_WALL_SNAP_OFFSET_IN_CLIMB_ZONE_TOLERANCE =
      Units.inchesToMeters(110.0);

  private static final Rectangle2d BLUE_OUTPOST_WALL_SNAP_CORNER_ZONE =
      new Rectangle2d(
          new Translation2d(0.0, 0.0),
          new Translation2d(WALL_SNAP_CORNER_ZONE_LENGTH, WALL_SNAP_CORNER_ZONE_LENGTH));
  private static final Rectangle2d BLUE_DEPOT_WALL_SNAP_CORNER_ZONE =
      new Rectangle2d(
          new Translation2d(0.0, FIELD_WIDTH_Y),
          new Translation2d(
              WALL_SNAP_CORNER_ZONE_LENGTH, FIELD_WIDTH_Y - WALL_SNAP_CORNER_ZONE_LENGTH));
  private static final Rectangle2d RED_DEPOT_WALL_SNAP_CORNER_ZONE =
      new Rectangle2d(
          new Translation2d(FIELD_LENGTH_X, 0.0),
          new Translation2d(
              FIELD_LENGTH_X - WALL_SNAP_CORNER_ZONE_LENGTH, WALL_SNAP_CORNER_ZONE_LENGTH));
  private static final Rectangle2d RED_OUTPOST_WALL_SNAP_CORNER_ZONE =
      new Rectangle2d(
          new Translation2d(FIELD_LENGTH_X, FIELD_WIDTH_Y),
          new Translation2d(
              FIELD_LENGTH_X - WALL_SNAP_CORNER_ZONE_LENGTH,
              FIELD_WIDTH_Y - WALL_SNAP_CORNER_ZONE_LENGTH));

  private static final List<Rectangle2d> WALL_SNAP_CORNER_ZONES =
      ImmutableList.of(
          BLUE_OUTPOST_WALL_SNAP_CORNER_ZONE,
          BLUE_DEPOT_WALL_SNAP_CORNER_ZONE,
          RED_DEPOT_WALL_SNAP_CORNER_ZONE,
          RED_OUTPOST_WALL_SNAP_CORNER_ZONE);

  // HOME FIELD ONLY; Modified field util for home field
  private static final double HOME_FIELD_RED_DEPOT_WALL_OFFSET = Units.inchesToMeters(229.0);
  private static final Pose2d HOME_FIELD_RED_DEPOT_TRENCH_CENTER =
      new Pose2d(
          RED_DEPOT_TRENCH_CENTER.getX(),
          FIELD_WIDTH_Y - HOME_FIELD_RED_DEPOT_WALL_OFFSET,
          Rotation2d.kZero);

  private static final Rectangle2d HOME_FIELD_RED_DEPOT_TRENCH_ZONE =
      new Rectangle2d(HOME_FIELD_RED_DEPOT_TRENCH_CENTER, TRENCH_LENGTH_X, TRENCH_LENGTH_Y);
  private static final Rectangle2d HOME_FIELD_RED_DEPOT_TRENCH_ASSIST_ZONE =
      new Rectangle2d(
          HOME_FIELD_RED_DEPOT_TRENCH_CENTER,
          TRENCH_ASSIST_ZONE_LENGTH_X,
          TRENCH_ASSIST_ZONE_LENGTH_Y);
  private static final Translation2d HOME_FIELD_RED_DEPOT_ALLIANCE_ZONE_TRENCH_MIDPOINT =
      new Translation2d(
          RED_DEPOT_ALLIANCE_ZONE_TRENCH_MIDPOINT.getX(),
          HOME_FIELD_RED_DEPOT_TRENCH_CENTER.getY());
  private static final Translation2d HOME_FIELD_RED_DEPOT_NEUTRAL_ZONE_TRENCH_MIDPOINT =
      new Translation2d(
          RED_DEPOT_NEUTRAL_ZONE_TRENCH_MIDPOINT.getX(), HOME_FIELD_RED_DEPOT_TRENCH_CENTER.getY());

  private static final List<Rectangle2d> HOME_FIELD_TRENCH_ZONES =
      ImmutableList.of(HOME_FIELD_RED_DEPOT_TRENCH_ZONE, RED_OUTPOST_TRENCH_ZONE);

  private static final List<Rectangle2d> HOME_FIELD_TRENCH_ASSIST_ZONES =
      ImmutableList.of(HOME_FIELD_RED_DEPOT_TRENCH_ASSIST_ZONE, RED_OUTPOST_TRENCH_ASSIST_ZONE);

  private static final List<Translation2d> HOME_FIELD_ALLIANCE_ZONE_TRENCH_MIDPOINTS =
      ImmutableList.of(
          HOME_FIELD_RED_DEPOT_ALLIANCE_ZONE_TRENCH_MIDPOINT,
          RED_OUTPOST_ALLIANCE_ZONE_TRENCH_MIDPOINT);

  private static final List<Translation2d> HOME_FIELD_NEUTRAL_ZONE_TRENCH_MIDPOINTS =
      ImmutableList.of(
          HOME_FIELD_RED_DEPOT_NEUTRAL_ZONE_TRENCH_MIDPOINT,
          RED_OUTPOST_NEUTRAL_ZONE_TRENCH_MIDPOINT);

  private static final double HOME_FIELD_LENGTH_X = Units.inchesToMeters(326.0);
  private static final double HOME_FIELD_WIDTH_Y = Units.inchesToMeters(253.5);
  public static final Rectangle2d HOME_FIELD_FIELD_BOUNDS =
      new Rectangle2d(
          new Translation2d(
              FIELD_LENGTH_X - HOME_FIELD_LENGTH_X, FIELD_WIDTH_Y - HOME_FIELD_WIDTH_Y),
          new Translation2d(FIELD_LENGTH_X, FIELD_WIDTH_Y));

  private static final Rectangle2d HOME_FIELD_BLUE_OUTPOST_WALL_SNAP_CORNER_ZONE =
      new Rectangle2d(
          new Translation2d(
              FIELD_LENGTH_X - HOME_FIELD_LENGTH_X, FIELD_WIDTH_Y - HOME_FIELD_WIDTH_Y),
          new Translation2d(
              FIELD_LENGTH_X - HOME_FIELD_LENGTH_X + WALL_SNAP_CORNER_ZONE_LENGTH,
              FIELD_WIDTH_Y - HOME_FIELD_WIDTH_Y + WALL_SNAP_CORNER_ZONE_LENGTH));
  private static final Rectangle2d HOME_FIELD_BLUE_DEPOT_WALL_SNAP_CORNER_ZONE =
      new Rectangle2d(
          new Translation2d(FIELD_LENGTH_X - HOME_FIELD_LENGTH_X, FIELD_WIDTH_Y),
          new Translation2d(
              FIELD_LENGTH_X - HOME_FIELD_LENGTH_X + WALL_SNAP_CORNER_ZONE_LENGTH,
              FIELD_WIDTH_Y - WALL_SNAP_CORNER_ZONE_LENGTH));
  private static final Rectangle2d HOME_FIELD_RED_DEPOT_WALL_SNAP_CORNER_ZONE =
      new Rectangle2d(
          new Translation2d(FIELD_LENGTH_X, FIELD_WIDTH_Y - HOME_FIELD_WIDTH_Y),
          new Translation2d(
              FIELD_LENGTH_X - WALL_SNAP_CORNER_ZONE_LENGTH,
              FIELD_WIDTH_Y - HOME_FIELD_WIDTH_Y + WALL_SNAP_CORNER_ZONE_LENGTH));

  private static final List<Rectangle2d> HOME_FIELD_WALL_SNAP_CORNER_ZONES =
      ImmutableList.of(
          HOME_FIELD_BLUE_OUTPOST_WALL_SNAP_CORNER_ZONE,
              HOME_FIELD_BLUE_DEPOT_WALL_SNAP_CORNER_ZONE,
          HOME_FIELD_RED_DEPOT_WALL_SNAP_CORNER_ZONE, RED_OUTPOST_WALL_SNAP_CORNER_ZONE);

  // TODO: Validate these points
  private static final Pose2d BLUE_LEFT_FALLBACK =
      new Pose2d(BLUE_OBSTACLE_X - TRENCH_LENGTH_X / 2.0, AprilTags.TAG_7.getY(), new Rotation2d());
  private static final Pose2d BLUE_RIGHT_FALLBACK =
      new Pose2d(
          BLUE_OBSTACLE_X - TRENCH_LENGTH_X / 2.0, AprilTags.TAG_12.getY(), new Rotation2d());
  private static final Pose2d RED_LEFT_FALLBACK =
      new Pose2d(RED_OBSTACLE_X + TRENCH_LENGTH_X / 2.0, AprilTags.TAG_17.getY(), new Rotation2d());
  private static final Pose2d RED_RIGHT_FALLBACK =
      new Pose2d(RED_OBSTACLE_X + TRENCH_LENGTH_X / 2.0, AprilTags.TAG_22.getY(), new Rotation2d());

  private static final Triangle2d RED_HUB_NO_FEED_ZONE =
      new Triangle2d(
          FIELD_BOUNDS.getCenter().getTranslation(),
          new Translation2d(
              HUB_POSE.redPose().getX() - HUB_RADIUS_METERS - Units.inchesToMeters(6),
              HUB_POSE.redPose().getY() + HUB_RADIUS_METERS + Units.inchesToMeters(8)),
          new Translation2d(
              HUB_POSE.redPose().getX() - HUB_RADIUS_METERS - Units.inchesToMeters(6),
              HUB_POSE.redPose().getY() - HUB_RADIUS_METERS - Units.inchesToMeters(8)));
  private static final Triangle2d BLUE_HUB_NO_FEED_ZONE =
      new Triangle2d(
          FIELD_BOUNDS.getCenter().getTranslation(),
          new Translation2d(
              HUB_POSE.bluePose().getX() + HUB_RADIUS_METERS + Units.inchesToMeters(6),
              HUB_POSE.bluePose().getY() + HUB_RADIUS_METERS + Units.inchesToMeters(8)),
          new Translation2d(
              HUB_POSE.bluePose().getX() + HUB_RADIUS_METERS + Units.inchesToMeters(6),
              HUB_POSE.bluePose().getY() - HUB_RADIUS_METERS - Units.inchesToMeters(8)));

  private static final Point CLIMB_FRONT_CORNER_OUTPOST_SIDE =
      Point.ofRed(
          new Pose2d(
              Units.inchesToMeters(608.375814),
              Units.inchesToMeters(146.718750 - 6.0),
              Rotation2d.kZero));
  private static final Point CLIMB_BACK_CORNER_DEPOT_SIDE =
      Point.ofRed(
          new Pose2d(FIELD_LENGTH_X, Units.inchesToMeters(193.718750 + 6.0), Rotation2d.kZero));

  private static final Rectangle2d RED_CLIMB_ZONE =
      new Rectangle2d(
          CLIMB_FRONT_CORNER_OUTPOST_SIDE.redPose().getTranslation(),
          CLIMB_BACK_CORNER_DEPOT_SIDE.redPose().getTranslation());
  private static final Rectangle2d BLUE_CLIMB_ZONE =
      new Rectangle2d(
          CLIMB_FRONT_CORNER_OUTPOST_SIDE.bluePose().getTranslation(),
          CLIMB_BACK_CORNER_DEPOT_SIDE.bluePose().getTranslation());

  // TODO: Define obstruction polygons and integrate with shooting logic
  public static final ObstructionCalculator FIELD_OBSTACLES =
      ObstructionCalculator.fromTranslations();

  public static Pose2d clampPoseToAllianceZone(Pose2d robot) {
    if (isRobotInAllianceZone(robot.getTranslation())) {
      return robot;
    }
    return new Pose2d(getAllianceZoneX(), robot.getY(), robot.getRotation());
  }

  // Logs all trench/bump zone corners & points; only needs to run once
  public static void debugLogFieldZones() {
    // Trenches
    DogLog.log(
        "FieldUtil/BlueOutpost/Trenches/Corner1",
        new Pose2d(MathHelpers.getCorners(BLUE_OUTPOST_TRENCH_ZONE).get(0), Rotation2d.kCW_90deg));
    DogLog.log(
        "FieldUtil/BlueOutpost/Trenches/Corner2",
        new Pose2d(MathHelpers.getCorners(BLUE_OUTPOST_TRENCH_ZONE).get(2), Rotation2d.kCW_90deg));
    DogLog.log(
        "FieldUtil/BlueDepot/Trenches/Corner1",
        new Pose2d(MathHelpers.getCorners(BLUE_DEPOT_TRENCH_ZONE).get(0), Rotation2d.kCW_90deg));
    DogLog.log(
        "FieldUtil/BlueDepot/Trenches/Corner2",
        new Pose2d(MathHelpers.getCorners(BLUE_DEPOT_TRENCH_ZONE).get(2), Rotation2d.kCW_90deg));
    DogLog.log(
        "FieldUtil/RedDepot/Trenches/Corner1",
        new Pose2d(MathHelpers.getCorners(RED_DEPOT_TRENCH_ZONE).get(0), Rotation2d.kCW_90deg));
    DogLog.log(
        "FieldUtil/RedDepot/Trenches/Corner2",
        new Pose2d(MathHelpers.getCorners(RED_DEPOT_TRENCH_ZONE).get(2), Rotation2d.kCW_90deg));
    DogLog.log(
        "FieldUtil/RedOutpost/Trenches/Corner1",
        new Pose2d(MathHelpers.getCorners(RED_OUTPOST_TRENCH_ZONE).get(0), Rotation2d.kCW_90deg));
    DogLog.log(
        "FieldUtil/RedOutpost/Trenches/Corner2",
        new Pose2d(MathHelpers.getCorners(RED_OUTPOST_TRENCH_ZONE).get(2), Rotation2d.kCW_90deg));

    // Trench assist zones
    DogLog.log(
        "FieldUtil/BlueOutpost/TrenchAssistZones/Corner1",
        new Pose2d(
            MathHelpers.getCorners(BLUE_OUTPOST_TRENCH_ASSIST_ZONE).get(0), Rotation2d.kCW_90deg));
    DogLog.log(
        "FieldUtil/BlueOutpost/TrenchAssistZones/Corner2",
        new Pose2d(
            MathHelpers.getCorners(BLUE_OUTPOST_TRENCH_ASSIST_ZONE).get(2), Rotation2d.kCW_90deg));
    DogLog.log(
        "FieldUtil/BlueDepot/TrenchAssistZones/Corner1",
        new Pose2d(
            MathHelpers.getCorners(BLUE_DEPOT_TRENCH_ASSIST_ZONE).get(0), Rotation2d.kCW_90deg));
    DogLog.log(
        "FieldUtil/BlueDepot/TrenchAssistZones/Corner2",
        new Pose2d(
            MathHelpers.getCorners(BLUE_DEPOT_TRENCH_ASSIST_ZONE).get(2), Rotation2d.kCW_90deg));
    DogLog.log(
        "FieldUtil/RedDepot/TrenchAssistZones/Corner1",
        new Pose2d(
            MathHelpers.getCorners(RED_DEPOT_TRENCH_ASSIST_ZONE).get(0), Rotation2d.kCW_90deg));
    DogLog.log(
        "FieldUtil/RedDepot/TrenchAssistZones/Corner2",
        new Pose2d(
            MathHelpers.getCorners(RED_DEPOT_TRENCH_ASSIST_ZONE).get(2), Rotation2d.kCW_90deg));
    DogLog.log(
        "FieldUtil/RedOutpost/TrenchAssistZones/Corner1",
        new Pose2d(
            MathHelpers.getCorners(RED_OUTPOST_TRENCH_ASSIST_ZONE).get(0), Rotation2d.kCW_90deg));
    DogLog.log(
        "FieldUtil/RedOutpost/TrenchAssistZones/Corner2",
        new Pose2d(
            MathHelpers.getCorners(RED_OUTPOST_TRENCH_ASSIST_ZONE).get(2), Rotation2d.kCW_90deg));

    // Trench assist points
    DogLog.log(
        "FieldUtil/BlueOutpost/TrenchAssistPoints/AllianceZoneTrenchMidpoint",
        new Pose2d(BLUE_OUTPOST_ALLIANCE_ZONE_TRENCH_MIDPOINT, Rotation2d.kZero));
    DogLog.log(
        "FieldUtil/BlueOutpost/TrenchAssistPoints/NeutralZoneTrenchMidpoint",
        new Pose2d(BLUE_OUTPOST_NEUTRAL_ZONE_TRENCH_MIDPOINT, Rotation2d.kZero));
    DogLog.log(
        "FieldUtil/BlueDepot/TrenchAssistPoints/AllianceZoneTrenchMidpoint",
        new Pose2d(BLUE_DEPOT_ALLIANCE_ZONE_TRENCH_MIDPOINT, Rotation2d.kZero));
    DogLog.log(
        "FieldUtil/BlueDepot/TrenchAssistPoints/NeutralZoneTrenchMidpoint",
        new Pose2d(BLUE_DEPOT_NEUTRAL_ZONE_TRENCH_MIDPOINT, Rotation2d.kZero));
    DogLog.log(
        "FieldUtil/RedDepot/TrenchAssistPoints/AllianceZoneTrenchMidpoint",
        new Pose2d(RED_DEPOT_ALLIANCE_ZONE_TRENCH_MIDPOINT, Rotation2d.kZero));
    DogLog.log(
        "FieldUtil/RedDepot/TrenchAssistPoints/NeutralZoneTrenchMidpoint",
        new Pose2d(RED_DEPOT_NEUTRAL_ZONE_TRENCH_MIDPOINT, Rotation2d.kZero));
    DogLog.log(
        "FieldUtil/RedOutpost/TrenchAssistPoints/AllianceZoneTrenchMidpoint",
        new Pose2d(RED_OUTPOST_ALLIANCE_ZONE_TRENCH_MIDPOINT, Rotation2d.kZero));
    DogLog.log(
        "FieldUtil/RedOutpost/TrenchAssistPoints/NeutralZoneTrenchMidpoint",
        new Pose2d(RED_OUTPOST_NEUTRAL_ZONE_TRENCH_MIDPOINT, Rotation2d.kZero));

    // Bump assist zones
    DogLog.log(
        "FieldUtil/BlueOutpost/BumpAssistZones/Corner1",
        new Pose2d(
            MathHelpers.getCorners(BLUE_OUTPOST_BUMP_ASSIST_ZONE).get(0), Rotation2d.kCW_90deg));
    DogLog.log(
        "FieldUtil/BlueOutpost/BumpAssistZones/Corner2",
        new Pose2d(
            MathHelpers.getCorners(BLUE_OUTPOST_BUMP_ASSIST_ZONE).get(2), Rotation2d.kCW_90deg));
    DogLog.log(
        "FieldUtil/BlueDepot/BumpAssistZones/Corner1",
        new Pose2d(
            MathHelpers.getCorners(BLUE_DEPOT_BUMP_ASSIST_ZONE).get(0), Rotation2d.kCW_90deg));
    DogLog.log(
        "FieldUtil/BlueDepot/BumpAssistZones/Corner2",
        new Pose2d(
            MathHelpers.getCorners(BLUE_DEPOT_BUMP_ASSIST_ZONE).get(2), Rotation2d.kCW_90deg));
    DogLog.log(
        "FieldUtil/RedDepot/BumpAssistZones/Corner1",
        new Pose2d(
            MathHelpers.getCorners(RED_DEPOT_BUMP_ASSIST_ZONE).get(0), Rotation2d.kCW_90deg));
    DogLog.log(
        "FieldUtil/RedDepot/BumpAssistZones/Corner2",
        new Pose2d(
            MathHelpers.getCorners(RED_DEPOT_BUMP_ASSIST_ZONE).get(2), Rotation2d.kCW_90deg));
    DogLog.log(
        "FieldUtil/RedOutpost/BumpAssistZones/Corner1",
        new Pose2d(
            MathHelpers.getCorners(RED_OUTPOST_BUMP_ASSIST_ZONE).get(0), Rotation2d.kCW_90deg));
    DogLog.log(
        "FieldUtil/RedOutpost/BumpAssistZones/Corner2",
        new Pose2d(
            MathHelpers.getCorners(RED_OUTPOST_BUMP_ASSIST_ZONE).get(2), Rotation2d.kCW_90deg));

    // Climb  zones
    DogLog.log(
        "FieldUtil/RedClimbZone/Corner1",
        new Pose2d(MathHelpers.getCorners(RED_CLIMB_ZONE).get(0), Rotation2d.kCW_90deg));
    DogLog.log(
        "FieldUtil/RedClimbZone/Corner2",
        new Pose2d(MathHelpers.getCorners(RED_CLIMB_ZONE).get(2), Rotation2d.kCW_90deg));

    DogLog.log(
        "FieldUtil/BlueClimbZone/Corner1",
        new Pose2d(MathHelpers.getCorners(BLUE_CLIMB_ZONE).get(0), Rotation2d.kCW_90deg));
    DogLog.log(
        "FieldUtil/BlueClimbZone/Corner2",
        new Pose2d(MathHelpers.getCorners(BLUE_CLIMB_ZONE).get(2), Rotation2d.kCW_90deg));

    // Bump assist points
    DogLog.log(
        "FieldUtil/BlueOutpost/BumpAssistPoints/HubSideBumpPoint",
        new Pose2d(BLUE_OUTPOST_HUB_SIDE_BUMP_POINT, Rotation2d.kZero));
    DogLog.log(
        "FieldUtil/BlueOutpost/BumpAssistPoints/TrenchSideBumpPoint",
        new Pose2d(BLUE_OUTPOST_TRENCH_SIDE_BUMP_POINT, Rotation2d.kZero));
    DogLog.log(
        "FieldUtil/BlueDepot/BumpAssistPoints/HubSideBumpPoint",
        new Pose2d(BLUE_DEPOT_HUB_SIDE_BUMP_POINT, Rotation2d.kZero));
    DogLog.log(
        "FieldUtil/BlueDepot/BumpAssistPoints/TrenchSideBumpPoint",
        new Pose2d(BLUE_DEPOT_TRENCH_SIDE_BUMP_POINT, Rotation2d.kZero));
    DogLog.log(
        "FieldUtil/RedDepot/BumpAssistPoints/HubSideBumpPoint",
        new Pose2d(RED_DEPOT_HUB_SIDE_BUMP_POINT, Rotation2d.kZero));
    DogLog.log(
        "FieldUtil/RedDepot/BumpAssistPoints/TrenchSideBumpPoint",
        new Pose2d(RED_DEPOT_TRENCH_SIDE_BUMP_POINT, Rotation2d.kZero));
    DogLog.log(
        "FieldUtil/RedOutpost/BumpAssistPoints/HubSideBumpPoint",
        new Pose2d(RED_OUTPOST_HUB_SIDE_BUMP_POINT, Rotation2d.kZero));
    DogLog.log(
        "FieldUtil/RedOutpost/BumpAssistPoints/TrenchSideBumpPoint",
        new Pose2d(RED_OUTPOST_TRENCH_SIDE_BUMP_POINT, Rotation2d.kZero));

    // Wall snap corner zones
    DogLog.log(
        "FieldUtil/BlueOutpost/WallSnapCornerZones/Corner1",
        new Pose2d(
            MathHelpers.getCorners(BLUE_OUTPOST_WALL_SNAP_CORNER_ZONE).get(0),
            Rotation2d.kCW_90deg));
    DogLog.log(
        "FieldUtil/BlueOutpost/WallSnapCornerZones/Corner2",
        new Pose2d(
            MathHelpers.getCorners(BLUE_OUTPOST_WALL_SNAP_CORNER_ZONE).get(2),
            Rotation2d.kCW_90deg));
    DogLog.log(
        "FieldUtil/BlueDepot/WallSnapCornerZones/Corner1",
        new Pose2d(
            MathHelpers.getCorners(BLUE_DEPOT_WALL_SNAP_CORNER_ZONE).get(1), Rotation2d.kCW_90deg));
    DogLog.log(
        "FieldUtil/BlueDepot/WallSnapCornerZones/Corner2",
        new Pose2d(
            MathHelpers.getCorners(BLUE_DEPOT_WALL_SNAP_CORNER_ZONE).get(3), Rotation2d.kCW_90deg));
    DogLog.log(
        "FieldUtil/RedDepot/WallSnapCornerZones/Corner1",
        new Pose2d(
            MathHelpers.getCorners(RED_DEPOT_WALL_SNAP_CORNER_ZONE).get(1), Rotation2d.kCW_90deg));
    DogLog.log(
        "FieldUtil/RedDepot/WallSnapCornerZones/Corner2",
        new Pose2d(
            MathHelpers.getCorners(RED_DEPOT_WALL_SNAP_CORNER_ZONE).get(3), Rotation2d.kCW_90deg));
    DogLog.log(
        "FieldUtil/RedOutpost/WallSnapCornerZones/Corner1",
        new Pose2d(
            MathHelpers.getCorners(RED_OUTPOST_WALL_SNAP_CORNER_ZONE).get(0),
            Rotation2d.kCW_90deg));
    DogLog.log(
        "FieldUtil/RedOutpost/WallSnapCornerZones/Corner2",
        new Pose2d(
            MathHelpers.getCorners(RED_OUTPOST_WALL_SNAP_CORNER_ZONE).get(2),
            Rotation2d.kCW_90deg));

    // No feed zones
    DogLog.log(
        "FieldUtil/BlueHubNoFeedZone/Corner1",
        new Pose2d(BLUE_HUB_NO_FEED_ZONE.getVertexA(), Rotation2d.kZero));
    DogLog.log(
        "FieldUtil/BlueHubNoFeedZone/Corner2",
        new Pose2d(BLUE_HUB_NO_FEED_ZONE.getVertexB(), Rotation2d.kZero));
    DogLog.log(
        "FieldUtil/BlueHubNoFeedZone/Corner3",
        new Pose2d(BLUE_HUB_NO_FEED_ZONE.getVertexC(), Rotation2d.kZero));
    DogLog.log(
        "FieldUtil/RedHubNoFeedZone/Corner1",
        new Pose2d(RED_HUB_NO_FEED_ZONE.getVertexA(), Rotation2d.kZero));
    DogLog.log(
        "FieldUtil/RedHubNoFeedZone/Corner2",
        new Pose2d(RED_HUB_NO_FEED_ZONE.getVertexB(), Rotation2d.kZero));
    DogLog.log(
        "FieldUtil/RedHubNoFeedZone/Corner3",
        new Pose2d(RED_HUB_NO_FEED_ZONE.getVertexC(), Rotation2d.kZero));

    // Home field red depot trench zones/points
    DogLog.log(
        "FieldUtil/HomeField/RedDepot/Trenches/Corner1",
        new Pose2d(
            MathHelpers.getCorners(HOME_FIELD_RED_DEPOT_TRENCH_ZONE).get(0), Rotation2d.kCW_90deg));
    DogLog.log(
        "FieldUtil/HomeField/RedDepot/Trenches/Corner2",
        new Pose2d(
            MathHelpers.getCorners(HOME_FIELD_RED_DEPOT_TRENCH_ZONE).get(2), Rotation2d.kCW_90deg));
    DogLog.log(
        "FieldUtil/HomeField/RedDepot/TrenchAssistZones/Corner1",
        new Pose2d(
            MathHelpers.getCorners(HOME_FIELD_RED_DEPOT_TRENCH_ASSIST_ZONE).get(0),
            Rotation2d.kCW_90deg));
    DogLog.log(
        "FieldUtil/HomeField/RedDepot/TrenchAssistZones/Corner2",
        new Pose2d(
            MathHelpers.getCorners(HOME_FIELD_RED_DEPOT_TRENCH_ASSIST_ZONE).get(2),
            Rotation2d.kCW_90deg));
    DogLog.log(
        "FieldUtil/HomeField/RedDepot/TrenchAssistPoints/AllianceZoneTrenchMidpoint",
        new Pose2d(HOME_FIELD_RED_DEPOT_ALLIANCE_ZONE_TRENCH_MIDPOINT, Rotation2d.kZero));
    DogLog.log(
        "FieldUtil/HomeField/RedDepot/TrenchAssistPoints/NeutralZoneTrenchMidpoint",
        new Pose2d(HOME_FIELD_RED_DEPOT_NEUTRAL_ZONE_TRENCH_MIDPOINT, Rotation2d.kZero));

    // Home field wall snap corner zones
    DogLog.log(
        "FieldUtil/HomeField/BlueOutpost/WallSnapCornerZone/Corner1",
        new Pose2d(
            MathHelpers.getCorners(HOME_FIELD_BLUE_OUTPOST_WALL_SNAP_CORNER_ZONE).get(0),
            Rotation2d.kCW_90deg));
    DogLog.log(
        "FieldUtil/HomeField/BlueOutpost/WallSnapCornerZone/Corner2",
        new Pose2d(
            MathHelpers.getCorners(HOME_FIELD_BLUE_OUTPOST_WALL_SNAP_CORNER_ZONE).get(2),
            Rotation2d.kCW_90deg));
    DogLog.log(
        "FieldUtil/HomeField/BlueDepot/WallSnapCornerZone/Corner1",
        new Pose2d(
            MathHelpers.getCorners(HOME_FIELD_BLUE_DEPOT_WALL_SNAP_CORNER_ZONE).get(1),
            Rotation2d.kCW_90deg));
    DogLog.log(
        "FieldUtil/HomeField/BlueDepot/WallSnapCornerZone/Corner2",
        new Pose2d(
            MathHelpers.getCorners(HOME_FIELD_BLUE_DEPOT_WALL_SNAP_CORNER_ZONE).get(3),
            Rotation2d.kCW_90deg));
    DogLog.log(
        "FieldUtil/HomeField/RedDepot/WallSnapCornerZone/Corner1",
        new Pose2d(
            MathHelpers.getCorners(HOME_FIELD_RED_DEPOT_WALL_SNAP_CORNER_ZONE).get(1),
            Rotation2d.kCW_90deg));
    DogLog.log(
        "FieldUtil/HomeField/RedDepot/WallSnapCornerZone/Corner2",
        new Pose2d(
            MathHelpers.getCorners(HOME_FIELD_RED_DEPOT_WALL_SNAP_CORNER_ZONE).get(3),
            Rotation2d.kCW_90deg));
    DogLog.log(
        "FieldUtil/HomeField/RedOutpost/WallSnapCornerZone/Corner1",
        new Pose2d(
            MathHelpers.getCorners(RED_OUTPOST_WALL_SNAP_CORNER_ZONE).get(0),
            Rotation2d.kCW_90deg));
    DogLog.log(
        "FieldUtil/HomeField/RedOutpost/WallSnapCornerZone/Corner2",
        new Pose2d(
            MathHelpers.getCorners(RED_OUTPOST_WALL_SNAP_CORNER_ZONE).get(2),
            Rotation2d.kCW_90deg));
  }

  public static double getAllianceZoneX() {
    return FmsUtil.isRedAlliance() ? RED_STARTING_LINE_X : BLUE_STARTING_LINE_X;
  }

  public static Translation2d getClosestAllianceZoneTrenchMidpoint(Translation2d robotTranslation) {
    return robotTranslation.nearest(ALLIANCE_ZONE_TRENCH_MIDPOINTS);
  }

  // HOME FIELD ONLY
  public static Translation2d getClosestHomeFieldAllianceZoneTrenchMidpoint(
      Translation2d robotTranslation) {
    return robotTranslation.nearest(HOME_FIELD_ALLIANCE_ZONE_TRENCH_MIDPOINTS);
  }

  // HOME FIELD ONLY
  public static Translation2d getClosestHomeFieldNeutralZoneTrenchMidpoint(
      Translation2d robotTranslation) {
    return robotTranslation.nearest(HOME_FIELD_NEUTRAL_ZONE_TRENCH_MIDPOINTS);
  }

  public static Translation2d getClosestHubSideBumpPoint(Translation2d robotTranslation) {
    return robotTranslation.nearest(HUB_SIDE_BUMP_POINTS);
  }

  public static Translation2d getClosestNeutralZoneTrenchMidpoint(Translation2d robotTranslation) {
    return robotTranslation.nearest(NEUTRAL_ZONE_TRENCH_MIDPOINTS);
  }

  public static Translation2d getClosestTrenchSideBumpPoint(Translation2d robotTranslation) {
    return robotTranslation.nearest(TRENCH_SIDE_BUMP_POINTS);
  }

  public static Optional<Rectangle2d> getCurrentBumpAssistZone(Translation2d robotTranslation) {
    return BUMP_ASSIST_ZONES.stream().filter(zone -> zone.contains(robotTranslation)).findFirst();
  }

  // HOME FIELD ONLY
  // Only check red outpost bump zone because that is the only one on the home field
  public static boolean getCurrentHomeFieldBumpAssistZone(Translation2d robotTranslation) {
    return RED_OUTPOST_BUMP_ASSIST_ZONE.contains(robotTranslation);
  }

  // HOME FIELD ONLY
  public static Optional<Rectangle2d> getCurrentHomeFieldTrenchAssistZone(
      Translation2d robotTranslation) {
    return HOME_FIELD_TRENCH_ASSIST_ZONES.stream()
        .filter(zone -> zone.contains(robotTranslation))
        .findFirst();
  }

  // HOME FIELD ONLY
  public static Optional<Rectangle2d> getCurrentHomeFieldWallSnapCornerZone(
      Translation2d robotTranslation) {
    return HOME_FIELD_WALL_SNAP_CORNER_ZONES.stream()
        .filter(zone -> zone.contains(robotTranslation))
        .findFirst();
  }

  /** Returns the trench assist zone that the robot is currently in, if it exists. */
  public static Optional<Rectangle2d> getCurrentTrenchAssistZone(Translation2d robotTranslation) {
    return TRENCH_ASSIST_ZONES.stream().filter(zone -> zone.contains(robotTranslation)).findFirst();
  }

  public static Optional<Rectangle2d> getCurrentWallSnapCornerZone(Translation2d robotTranslation) {
    return WALL_SNAP_CORNER_ZONES.stream()
        .filter(zone -> zone.contains(robotTranslation))
        .findFirst();
  }

  public static Pose2d getFallbackScorePoint() {
    var location = DriverStation.getLocation().orElse(1);

    if (FmsUtil.isRedAlliance()) {
      if (location == 1) {
        return RED_LEFT_FALLBACK;
      }

      return RED_RIGHT_FALLBACK;
    }

    if (location == 3) {
      return BLUE_LEFT_FALLBACK;
    }

    return BLUE_RIGHT_FALLBACK;
  }

  public static double getObstacleX() {
    return FmsUtil.isRedAlliance() ? RED_OBSTACLE_X : BLUE_OBSTACLE_X;
  }

  // HOME FIELD ONLY
  public static boolean inHomeFieldTrench(Translation2d robotPose) {
    return HOME_FIELD_TRENCH_ZONES.stream().anyMatch(zone -> zone.contains(robotPose));
  }

  public static boolean inTrench(Translation2d robotPose) {
    return TRENCH_ZONES.stream().anyMatch(zone -> zone.contains(robotPose));
  }

  public static boolean isInNoScoreZone(Pose2d robot) {
    if (FmsUtil.isRedAlliance()) {
      return RED_CLIMB_ZONE.contains(robot.getTranslation());
    }
    return BLUE_CLIMB_ZONE.contains(robot.getTranslation());
  }

  public static boolean isRobotInAllianceZone(Translation2d robot) {
    if (FmsUtil.isRedAlliance()) {
      return robot.getX() > getAllianceZoneX();
    }
    return robot.getX() < getAllianceZoneX();
  }

  public static boolean isRobotInNoFeedZone(Pose2d robotPose) {
    // Check if line from robot to target collides with hub no feed zone
    var noFeedZone = FmsUtil.isRedAlliance() ? RED_HUB_NO_FEED_ZONE : BLUE_HUB_NO_FEED_ZONE;

    return noFeedZone.contains(robotPose.getTranslation());
  }

  public static boolean isRobotPastObstacleTowardAllianceZone(Translation2d robot) {
    if (FmsUtil.isRedAlliance()) {
      return robot.getX() > getObstacleX();
    }
    return robot.getX() < getObstacleX();
  }

  // For wall snaps, can't apply offset toward wall under climb tower
  public static boolean nearClimbZone(Translation2d robotTranslation) {
    // Based on alliance field side because the climb zones are not bilaterally symmetrical
    var climbZone =
        robotTranslation.getX() < FieldUtil.FIELD_LENGTH_X / 2.0 ? BLUE_CLIMB_ZONE : RED_CLIMB_ZONE;
    return new Rectangle2d(
            climbZone.getCenter(),
            climbZone.getXWidth(),
            NO_WALL_SNAP_OFFSET_IN_CLIMB_ZONE_TOLERANCE)
        .contains(robotTranslation);
  }

  /**
   * Returns the input pose flipped from red to blue (or vice versa).
   *
   * @param input Pose to transform
   */
  public static Pose2d pathflip(Pose2d input) {
    return input.rotateAround(FIELD_BOUNDS.getCenter().getTranslation(), Rotation2d.k180deg);
  }
}
