package com.team581.math;

import com.google.common.collect.ImmutableList;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.Collections;
import java.util.List;

public class MathHelpers {
  private static final double EPSILON = Math.ulp(1.0);

  public static double angleModulus(double angleDegrees) {
    return MathUtil.inputModulus(angleDegrees, -180, 180);
  }

  public static double average(double a, double b) {
    return (a + b) / 2.0;
  }

  /**
   * Returns the value that is farther from the two given values. If they are equal, the first value
   * is returned.
   */
  public static double farthest(double value, double a, double b) {
    return Math.abs(value - a) >= Math.abs(value - b) ? a : b;
  }

  /**
   * Returns the closest point on the perimeter of a rectangle to a given point. Modification of
   * built in WPILib function to get nearest point
   */
  public static final Translation2d getClosestPointOnRectanglePerimeter(
      Translation2d point, Rectangle2d rectangle) {
    // Rotate point by inverse of rectangle rotation
    var m_center = rectangle.getCenter();
    var m_xWidth = rectangle.getXWidth();
    var m_yWidth = rectangle.getYWidth();
    var rotatedPoint =
        point.rotateAround(m_center.getTranslation(), m_center.getRotation().unaryMinus());

    // Find nearest point on or inside the rectangle
    double minX = m_center.getX() - m_xWidth / 2.0;
    double maxX = m_center.getX() + m_xWidth / 2.0;
    double minY = m_center.getY() - m_yWidth / 2.0;
    double maxY = m_center.getY() + m_yWidth / 2.0;

    var nearestPoint =
        new Translation2d(
            MathUtil.clamp(rotatedPoint.getX(), minX, maxX),
            MathUtil.clamp(rotatedPoint.getY(), minY, maxY));

    // If point inside rectangle push it to the perimeter
    if (nearestPoint.getX() > minX
        && nearestPoint.getX() < maxX
        && nearestPoint.getY() > minY
        && nearestPoint.getY() < maxY) {
      double dxMin = nearestPoint.getX() - minX;
      double dxMax = maxX - nearestPoint.getX();
      double dyMin = nearestPoint.getY() - minY;
      double dyMax = maxY - nearestPoint.getY();

      double minDist = Math.min(Math.min(dxMin, dxMax), Math.min(dyMin, dyMax));

      if (minDist == dxMin) {
        nearestPoint = new Translation2d(minX, nearestPoint.getY());
      } else if (minDist == dxMax) {
        nearestPoint = new Translation2d(maxX, nearestPoint.getY());
      } else if (minDist == dyMin) {
        nearestPoint = new Translation2d(nearestPoint.getX(), minY);
      } else {
        nearestPoint = new Translation2d(nearestPoint.getX(), maxY);
      }
    }

    // Undo rotation
    return nearestPoint.rotateAround(m_center.getTranslation(), m_center.getRotation());
  }

  public static final List<Translation2d> getCorners(Rectangle2d rectangle) {
    var center = rectangle.getCenter();
    var xWidth = rectangle.getXWidth();
    var yWidth = rectangle.getYWidth();
    return ImmutableList.of(
        new Translation2d(center.getX() - xWidth / 2.0, center.getY() - yWidth / 2.0),
        new Translation2d(center.getX() + xWidth / 2.0, center.getY() - yWidth / 2.0),
        new Translation2d(center.getX() + xWidth / 2.0, center.getY() + yWidth / 2.0),
        new Translation2d(center.getX() - xWidth / 2.0, center.getY() + yWidth / 2.0));
  }

  public static Rotation2d getDriveDirection(ChassisSpeeds vector) {
    if (getLinearVelocity(vector) < 1e-6) {
      return Rotation2d.kZero;
    }

    return new Translation2d(vector.vxMetersPerSecond, vector.vyMetersPerSecond).getAngle();
  }

  public static Rotation2d getDriveDirection(Pose2d start, Pose2d end) {
    return getDriveDirection(start.getTranslation(), end.getTranslation());
  }

  public static Rotation2d getDriveDirection(Pose2d start, Translation2d end) {
    return getDriveDirection(start.getTranslation(), end);
  }

  public static Rotation2d getDriveDirection(Translation2d start, Pose2d end) {
    return getDriveDirection(start, end.getTranslation());
  }

  public static Rotation2d getDriveDirection(Translation2d start, Translation2d end) {
    return rotation2d(end.getX() - start.getX(), end.getY() - start.getY());
  }

  /**
   * Returns the intersection point on the perimeter of a rectangle from a given point and angle.
   */
  public static final Translation2d getIntersectionOnRectanglePerimeter(
      Translation2d point, Rectangle2d rectangle, Rotation2d angle) {

    // Rotate point and angle by inverse of rectangle rotation to align with axes
    var m_center = rectangle.getCenter();
    var m_xWidth = rectangle.getXWidth();
    var m_yWidth = rectangle.getYWidth();
    var rotatedPoint =
        point.rotateAround(m_center.getTranslation(), m_center.getRotation().unaryMinus());
    var rotatedAngle = angle.minus(m_center.getRotation());

    // Find intersection with axis-aligned rectangle
    double minX = m_center.getX() - m_xWidth / 2.0;
    double maxX = m_center.getX() + m_xWidth / 2.0;
    double minY = m_center.getY() - m_yWidth / 2.0;
    double maxY = m_center.getY() + m_yWidth / 2.0;

    double pointX = rotatedPoint.getX();
    double pointY = rotatedPoint.getY();
    double cos = rotatedAngle.getCos();
    double sin = rotatedAngle.getSin();

    var intersectedValues = new java.util.ArrayList<Double>();

    // Check for intersection with each side
    if (Math.abs(cos) > 1e-9) {
      double intersect1 = (minX - pointX) / cos;
      if (intersect1 >= 0) {
        double y = pointY + intersect1 * sin;
        if (y >= minY && y <= maxY) {
          intersectedValues.add(intersect1);
        }
      }

      double intersect2 = (maxX - pointX) / cos;
      if (intersect2 >= 0) {
        double y = pointY + intersect2 * sin;
        if (y >= minY && y <= maxY) {
          intersectedValues.add(intersect2);
        }
      }
    }

    if (Math.abs(sin) > 1e-9) {
      double intersect3 = (minY - pointY) / sin;
      if (intersect3 >= 0) {
        double x = pointX + intersect3 * cos;
        if (x >= minX && x <= maxX) {
          intersectedValues.add(intersect3);
        }
      }

      double intersect4 = (maxY - pointY) / sin;
      if (intersect4 >= 0) {
        double x = pointX + intersect4 * cos;
        if (x >= minX && x <= maxX) {
          intersectedValues.add(intersect4);
        }
      }
    }

    if (intersectedValues.isEmpty()) {
      // No intersection found, return closest point on perimeter instead
      return getClosestPointOnRectanglePerimeter(point, rectangle);
    }

    double minIntersected = Collections.min(intersectedValues);
    var intersectionPoint =
        new Translation2d(pointX + minIntersected * cos, pointY + minIntersected * sin);

    // Undo rotation
    return intersectionPoint.rotateAround(m_center.getTranslation(), m_center.getRotation());
  }

  public static double getLinearVelocity(ChassisSpeeds speeds) {
    return Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
  }

  public static final Pose2d getLookaheadPose(
      Pose2d currentPose, ChassisSpeeds speeds, double time) {
    var x = currentPose.getX() + speeds.vxMetersPerSecond * time;
    var y = currentPose.getY() + speeds.vyMetersPerSecond * time;
    var theta =
        currentPose.getRotation().plus(Rotation2d.fromRadians(speeds.omegaRadiansPerSecond * time));

    return new Pose2d(x, y, theta);
  }

  /**
   * Perform linear interpolation between two ChassisSpeeds.
   *
   * @param startValue The ChassisSpeeds to start at.
   * @param endValue The ChassisSpeeds to end at.
   * @param t How far between the two ChassisSpeeds to interpolate. This is clamped to [0, 1].
   * @return The interpolated ChassisSpeeds.
   */
  public static ChassisSpeeds interpolate(
      ChassisSpeeds startValue, ChassisSpeeds endValue, double t) {
    return startValue.plus(endValue.minus(startValue).times(MathUtil.clamp(t, 0, 1)));
  }

  /**
   * Returns the value that is closer from the two given values. If they are equal, the first value
   * is returned.
   */
  public static double nearest(double value, double a, double b) {
    return Math.abs(value - a) < Math.abs(value - b) ? a : b;
  }

  /**
   * Constructs a Rotation2d with the given x and y (cosine and sine) components.
   *
   * @param x The x component or cosine of the rotation.
   * @param y The y component or sine of the rotation.
   */
  public static Rotation2d rotation2d(double x, double y) {
    if (Math.hypot(x, y) < 1e-6) {
      return Rotation2d.kZero;
    }

    return new Rotation2d(x, y);
  }

  public static Translation2d roundTo(Translation2d input, double precision) {
    return new Translation2d(roundTo(input.getX(), precision), roundTo(input.getY(), precision));
  }

  /**
   * Returns a value rounded to the specified number of decimal places.
   *
   * @param value The value
   * @param numDigits The number of digits after the decimal point to include
   */
  public static double roundTo(double value, double numDigits) {
    var factor = Math.pow(10, numDigits);

    return Math.round(value * factor * (1 + EPSILON)) / factor;
  }

  public static double signedSqrt(double value) {
    return Math.copySign(Math.sqrt(Math.abs(value)), value);
  }

  public static final Transform2d transform2dFromRotation(Rotation2d rotation) {
    return new Transform2d(Translation2d.kZero, rotation);
  }

  private MathHelpers() {}
}
