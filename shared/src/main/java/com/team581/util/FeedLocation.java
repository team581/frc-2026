package com.team581.util;

import com.team581.autos.Point;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public enum FeedLocation {
  LEFT(FieldUtil.FEED_LEFT_POSE),
  RIGHT(FieldUtil.FEED_RIGHT_POSE),
  BACKUP_LEFT(FieldUtil.BACKUP_FEED_LEFT_POSE),
  BACKUP_RIGHT(FieldUtil.BACKUP_FEED_RIGHT_POSE);

  public static FeedLocation closest(Pose2d robot) {
    var yDistanceToLeft = Math.abs(robot.getY() - FieldUtil.FEED_LEFT_POSE.getY());
    var yDistanceToRight = Math.abs(robot.getY() - FieldUtil.FEED_RIGHT_POSE.getY());
    var closestFeedLocation = Math.min(yDistanceToLeft, yDistanceToRight);
    if (closestFeedLocation == yDistanceToLeft) {
      if (FieldUtil.isFeedPathObstructed(
          robot.getTranslation(), FieldUtil.FEED_LEFT_POSE.getTranslation())) {
        return FeedLocation.BACKUP_LEFT;
      }
      return FeedLocation.LEFT;
    }
    if (FieldUtil.isFeedPathObstructed(
        robot.getTranslation(), FieldUtil.FEED_RIGHT_POSE.getTranslation())) {
      return FeedLocation.BACKUP_RIGHT;
    }
    return FeedLocation.RIGHT;
  }

  public final Point point;

  FeedLocation(Point point) {
    this.point = point;
  }

  public Translation2d getTranslation() {
    return point.getTranslation();
  }
}
