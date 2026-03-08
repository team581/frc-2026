package com.team581.math;

import static org.assertj.core.api.Assertions.assertThat;

import com.google.common.collect.ImmutableList;
import edu.wpi.first.math.geometry.Translation2d;
import org.junit.jupiter.api.Test;

final class ObstructionCalculatorTest {
  private static final ObstructionCalculator CALCULATOR =
      ObstructionCalculator.fromTranslations(
          ImmutableList.of(
              new Translation2d(0, 0),
              new Translation2d(2, 0),
              new Translation2d(2, 2),
              new Translation2d(0, 2)));

  @Test
  void multipleObstructions() {
    var calculator =
        ObstructionCalculator.fromTranslations(
            ImmutableList.of(
                new Translation2d(0, 0),
                new Translation2d(1, 0),
                new Translation2d(1, 1),
                new Translation2d(0, 1)),
            ImmutableList.of(
                new Translation2d(5, 5),
                new Translation2d(6, 5),
                new Translation2d(6, 6),
                new Translation2d(5, 6)));

    assertThat(calculator.contains(new Translation2d(0.5, 0.5))).isTrue();
    assertThat(calculator.contains(new Translation2d(5.5, 5.5))).isTrue();
    assertThat(calculator.contains(new Translation2d(3, 3))).isFalse();
  }

  @Test
  void pointInsideObstructionContains() {
    assertThat(CALCULATOR.contains(new Translation2d(1, 1))).isTrue();
  }

  @Test
  void pointOnCornerContains() {
    assertThat(CALCULATOR.contains(new Translation2d(0, 0))).isTrue();
  }

  @Test
  void pointOnEdgeContains() {
    assertThat(CALCULATOR.contains(new Translation2d(1, 0))).isTrue();
  }

  @Test
  void pointOutsideObstructionIsNotObstructed() {
    assertThat(CALCULATOR.contains(new Translation2d(5, 5))).isFalse();
  }

  @Test
  void segmentBetweenObstructionsIsNotObstructed() {
    var calculator =
        ObstructionCalculator.fromTranslations(
            ImmutableList.of(
                new Translation2d(0, 0),
                new Translation2d(1, 0),
                new Translation2d(1, 1),
                new Translation2d(0, 1)),
            ImmutableList.of(
                new Translation2d(3, 0),
                new Translation2d(4, 0),
                new Translation2d(4, 1),
                new Translation2d(3, 1)));

    assertThat(calculator.contains(new Translation2d(1.5, 0.5), new Translation2d(2.5, 0.5)))
        .isFalse();
  }

  @Test
  void segmentEndingAtEdgeContains() {
    assertThat(CALCULATOR.contains(new Translation2d(-1, 1), new Translation2d(0, 1))).isTrue();
  }

  @Test
  void segmentEntirelyInsideObstructionContains() {
    assertThat(CALCULATOR.contains(new Translation2d(0.5, 0.5), new Translation2d(1.5, 1.5)))
        .isTrue();
  }

  @Test
  void segmentOutsideObstructionIsNotObstructed() {
    assertThat(CALCULATOR.contains(new Translation2d(3, 3), new Translation2d(5, 5))).isFalse();
  }

  @Test
  void segmentStartingInsideContains() {
    assertThat(CALCULATOR.contains(new Translation2d(1, 1), new Translation2d(5, 5))).isTrue();
  }

  @Test
  void segmentThroughObstructionContains() {
    assertThat(CALCULATOR.contains(new Translation2d(-1, 1), new Translation2d(3, 1))).isTrue();
  }
}
