package com.team581.math;

import static org.assertj.core.api.Assertions.assertThat;
import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

final class Triangle2dTest {

  private static final double EPSILON = 1E-9; // For floating point comparisons
  private Triangle2d triangle;

  @Test
  void constructorAndGetters() {
    // Given
    Translation2d vertexA = new Translation2d(1.0, 2.0);
    Translation2d vertexB = new Translation2d(3.0, 4.0);
    Translation2d vertexC = new Translation2d(5.0, 6.0);

    // When
    Triangle2d aTriangle = new Triangle2d(vertexA, vertexB, vertexC);

    // Then
    assertEquals(vertexA.getX(), aTriangle.getVertexA().getX(), EPSILON);
    assertEquals(vertexA.getY(), aTriangle.getVertexA().getY(), EPSILON);

    assertEquals(vertexB.getX(), aTriangle.getVertexB().getX(), EPSILON);
    assertEquals(vertexB.getY(), aTriangle.getVertexB().getY(), EPSILON);

    assertEquals(vertexC.getX(), aTriangle.getVertexC().getX(), EPSILON);
    assertEquals(vertexC.getY(), aTriangle.getVertexC().getY(), EPSILON);
  }

  @Test
  void getClosestPoint_withPointInside_returnsPoint() {
    Translation2d insidePoint = new Translation2d(2, 1);
    Translation2d closestPoint = triangle.getClosestPoint(insidePoint);
    assertEquals(insidePoint.getX(), closestPoint.getX(), EPSILON);
    assertEquals(insidePoint.getY(), closestPoint.getY(), EPSILON);
  }

  @Test
  void getClosestPoint_withPointOutsideClosestToEdge_returnsPointOnEdge() {
    Translation2d outsidePoint = new Translation2d(2, -1);
    Translation2d closestPoint = triangle.getClosestPoint(outsidePoint);
    assertEquals(2, closestPoint.getX(), EPSILON);
    assertEquals(0, closestPoint.getY(), EPSILON);
  }

  @Test
  void getClosestPoint_withPointOutsideClosestToVertex_returnsVertex() {
    Translation2d outsidePoint = new Translation2d(-1, -1);
    Translation2d closestPoint = triangle.getClosestPoint(outsidePoint);
    assertEquals(0, closestPoint.getX(), EPSILON);
    assertEquals(0, closestPoint.getY(), EPSILON);
  }

  @Test
  void isInside_withPointInside_returnsTrue() {
    assertThat(triangle.contains(new Translation2d(2, 1))).isTrue();
  }

  @Test
  void isInside_withPointOnEdge_returnsTrue() {
    assertThat(triangle.contains(new Translation2d(2, 0))).isTrue();
  }

  @Test
  void isInside_withPointOnVertex_returnsTrue() {
    assertThat(triangle.contains(new Translation2d(0, 0))).isTrue();
  }

  @Test
  void isInside_withPointOutside_returnsFalse() {
    assertThat(triangle.contains(new Translation2d(5, 1))).isFalse();
  }

  @Test
  void isInside_withPoseInside_returnsTrue() {
    assertThat(triangle.isInside(new Pose2d(2, 1, null))).isTrue();
  }

  @BeforeEach
  void setUp() {
    Translation2d vertexA = new Translation2d(0, 0);
    Translation2d vertexB = new Translation2d(4, 0);
    Translation2d vertexC = new Translation2d(2, 3);
    triangle = new Triangle2d(vertexA, vertexB, vertexC);
  }
}
