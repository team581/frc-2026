package com.team581.math;

import static org.junit.jupiter.api.Assertions.assertEquals;

import com.team581.config.CameraConfig;
import com.team581.config.LimelightModel;
import com.team581.vision.results.GamePieceResult;
import edu.wpi.first.math.util.Units;
import org.junit.jupiter.api.Test;

final class GamePieceDetectionCalculatorTest {
  @Test
  void centeredDistanceCalculation() {
    var cameraConfig =
        new CameraConfig(
            LimelightModel.THREE, false, false, 0, 0, Units.inchesToMeters(10), 0, 0, 0);

    // Calculated from
    // https://www.calculator.net/triangle-calculator.html?vc=60&vx=10&vy=&va=90&vz=&vb=&angleunits=d&x=Calculate
    var expectedForwardDistance = Units.inchesToMeters(17.32051);
    var result = new GamePieceResult();
    result.update(0.0, -30.0, 0.0);
    var calculatedForwardDistance =
        GamePieceDetectionCalculator.calculateRobotRelativeTranslationFromCamera(
                result, cameraConfig)
            .getX();

    assertEquals(expectedForwardDistance, calculatedForwardDistance, Units.inchesToMeters(0.01));
  }
}
