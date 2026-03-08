package com.team581.math;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Triangle2d {
  private final Translation2d m_vertexA;
  private final Translation2d m_vertexB;
  private final Translation2d m_vertexC;

  /**
   * Constructs a Triangle2d from three Translation2d vertices.
   *
   * @param vertexA The first vertex of the triangle.
   * @param vertexB The second vertex of the triangle.
   * @param vertexC The third vertex of the triangle.
   */
  public Triangle2d(Translation2d vertexA, Translation2d vertexB, Translation2d vertexC) {
    this.m_vertexA = vertexA;
    this.m_vertexB = vertexB;
    this.m_vertexC = vertexC;
  }

  /**
   * Checks if a point is inside the triangle.
   *
   * @param point The point to check.
   * @return True if the point is inside the triangle, false otherwise.
   */
  public boolean contains(Translation2d point) {
    double s =
        (m_vertexA.getX() - m_vertexC.getX()) * (point.getY() - m_vertexC.getY())
            - (m_vertexA.getY() - m_vertexC.getY()) * (point.getX() - m_vertexC.getX());
    double t =
        (m_vertexB.getX() - m_vertexA.getX()) * (point.getY() - m_vertexA.getY())
            - (m_vertexB.getY() - m_vertexA.getY()) * (point.getX() - m_vertexA.getX());

    if ((s < 0) != (t < 0) && s != 0 && t != 0) {
      return false;
    }

    double A =
        (m_vertexC.getX() - m_vertexB.getX()) * (point.getY() - m_vertexB.getY())
            - (m_vertexC.getY() - m_vertexB.getY()) * (point.getX() - m_vertexB.getX());

    return s == 0 || t == 0 || A == 0 || (s > 0 && t > 0 && A > 0) || (s < 0 && t < 0 && A < 0);
  }

  /**
   * Returns the closest point on the triangle's perimeter to a given point.
   *
   * @param point The point to find the closest point to.
   * @return The closest point on the triangle's perimeter.
   */
  public Translation2d getClosestPoint(Translation2d point) {
    if (contains(point)) {
      return point;
    }

    Translation2d closestPoint = null;
    double minDistance = Double.POSITIVE_INFINITY;

    Translation2d[] vertices = {m_vertexA, m_vertexB, m_vertexC};
    for (int i = 0; i < 3; i++) {
      Translation2d p1 = vertices[i];
      Translation2d p2 = vertices[(i + 1) % 3];

      double dx = p2.getX() - p1.getX();
      double dy = p2.getY() - p1.getY();

      if (dx == 0 && dy == 0) {
        continue;
      }

      double t =
          ((point.getX() - p1.getX()) * dx + (point.getY() - p1.getY()) * dy) / (dx * dx + dy * dy);
      t = Math.max(0, Math.min(1, t));

      Translation2d projection = new Translation2d(p1.getX() + t * dx, p1.getY() + t * dy);
      double distance = projection.getDistance(point);

      if (distance < minDistance) {
        minDistance = distance;
        closestPoint = projection;
      }
    }

    return closestPoint;
  }

  /**
   * Returns the first vertex of the triangle.
   *
   * @return The first vertex.
   */
  public Translation2d getVertexA() {
    return m_vertexA;
  }

  /**
   * Returns the second vertex of the triangle.
   *
   * @return The second vertex.
   */
  public Translation2d getVertexB() {
    return m_vertexB;
  }

  /**
   * Returns the third vertex of the triangle.
   *
   * @return The third vertex.
   */
  public Translation2d getVertexC() {
    return m_vertexC;
  }

  /**
   * Checks if a pose is inside the triangle.
   *
   * @param pose The pose to check.
   * @return True if the pose is inside the triangle, false otherwise.
   */
  public boolean isInside(Pose2d pose) {
    return contains(pose.getTranslation());
  }
}
