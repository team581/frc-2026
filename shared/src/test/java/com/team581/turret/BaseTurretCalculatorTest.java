package com.team581.turret;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;

import com.team581.math.BaseTurretCalculator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.junit.jupiter.api.Test;

final class BaseTurretCalculatorTest {
  @Test
  void calculateHomedPositionFromMotorAndEncoder1() {
    var homedPosition =
        BaseTurretCalculator.calculateHomedPositionFromMotorAndEncoder(
            17.4, 0.001, 0.0, 40.0, 1.5, (1.0 / 40.0), -270, 270.0);
    assertEquals(0.01, homedPosition, 1e-9);
  }

  @Test
  void calculateHomedPositionFromMotorAndEncoder2() {
    var homedPosition =
        BaseTurretCalculator.calculateHomedPositionFromMotorAndEncoder(
            0.4, 0.0, 0.0, 40.0, 1.5, (1.0 / 40.0), -270, 270.0);
    assertEquals(0.01, homedPosition, 1e-9);
  }

  @Test
  void calculateHomedPositionFromMotorAndEncoder3() {
    var homedPosition =
        BaseTurretCalculator.calculateHomedPositionFromMotorAndEncoder(
            0.4, -0.001, 0.0, 40.0, 1.5, (1.0 / 40.0), -270, 270.0);
    assertEquals(0.01, homedPosition, 1e-9);
  }

  @Test
  void calculateHomedPositionFromMotorAndEncoder4() {
    var homedPosition =
        BaseTurretCalculator.calculateHomedPositionFromMotorAndEncoder(
            0.2, -0.5, 0.0, 40.0, 1.5, (1.0 / 40.0), -270, 270.0);
    assertEquals(-0.745, homedPosition, 1e-9);
  }

  @Test
  void calculateHomedPositionFromMotorAndEncoder5() {
    var homedPosition =
        BaseTurretCalculator.calculateHomedPositionFromMotorAndEncoder(
            0.75, 0.95, 0.0, 40.0, 1.5, (1.0 / 40.0), -270, 270.0);
    assertEquals(1.41875, homedPosition, 1e-9);
  }

  @Test
  void calculateSwerveTurretCompensationAngleNegative() {
    var actual =
        BaseTurretCalculator.calculateSwerveTurretCompensationAngle(
            -45.0, Rotation2d.fromDegrees(45.0), -180.0, 180.0);
    var expected = -240.0;
    assertEquals(expected, actual, 1e-9);
  }

  @Test
  void calculateSwerveTurretCompensationAnglePositive() {
    var actual =
        BaseTurretCalculator.calculateSwerveTurretCompensationAngle(
            45.0, Rotation2d.fromDegrees(45.0), -180.0, 180.0);
    var expected = 330.0;
    assertEquals(expected, actual, 1e-9);
  }

  @Test
  void calculateTurretAimingAngleNegative() {
    var robot = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));
    var target = new Translation2d(0.0, -1.0);

    var actual = BaseTurretCalculator.calculateTurretAimingAngle(robot, target, Transform2d.kZero);

    var expected = -90.0;
    assertEquals(expected, actual, 1e-9);
  }

  @Test
  void calculateTurretAimingAngleNegativeWithNegativeRobotRotation() {
    var robot = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(-45.0));
    var target = new Translation2d(0.0, -1.0);

    var actual = BaseTurretCalculator.calculateTurretAimingAngle(robot, target, Transform2d.kZero);

    var expected = -45.0;
    assertEquals(expected, actual, 1e-9);
  }

  @Test
  void calculateTurretAimingAngleNegativeWithPositiveRobotRotation() {
    var robot = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(45.0));
    var target = new Translation2d(0.0, -1.0);

    var actual = BaseTurretCalculator.calculateTurretAimingAngle(robot, target, Transform2d.kZero);

    var expected = -135.0;
    assertEquals(expected, actual, 1e-9);
  }

  @Test
  void calculateTurretAimingAnglePositive() {
    var robot = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));
    var target = new Translation2d(0.0, 1.0);

    var actual = BaseTurretCalculator.calculateTurretAimingAngle(robot, target, Transform2d.kZero);

    var expected = 90.0;
    assertEquals(expected, actual, 1e-9);
  }

  @Test
  void calculateTurretAimingAnglePositiveWithNegativeRobotRotation() {
    var robot = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(-45.0));
    var target = new Translation2d(0.0, 1.0);

    var actual = BaseTurretCalculator.calculateTurretAimingAngle(robot, target, Transform2d.kZero);

    var expected = 135.0;
    assertEquals(expected, actual, 1e-9);
  }

  @Test
  void calculateTurretAimingAnglePositiveWithPositiveRobotRotation() {
    var robot = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(45.0));
    var target = new Translation2d(0.0, 1.0);

    var actual = BaseTurretCalculator.calculateTurretAimingAngle(robot, target, Transform2d.kZero);

    var expected = 45.0;
    assertEquals(expected, actual, 1e-9);
  }

  @Test
  void doesTurretHaveRoomNegative() {
    var actual = BaseTurretCalculator.doesTurretHaveRoom(-90.0, -180.0, 180.0, 10.0, 5.0);
    assertEquals(true, actual);
  }

  @Test
  void doesTurretHaveRoomNegativeAtLimit() {
    var actual = BaseTurretCalculator.doesTurretHaveRoom(-170.0, -180.0, 180.0, 10.0, 5.0);
    assertEquals(true, actual);
  }

  @Test
  void doesTurretHaveRoomPositive() {
    var actual = BaseTurretCalculator.doesTurretHaveRoom(90.0, -180.0, 180.0, 10.0, 5.0);
    assertEquals(true, actual);
  }

  @Test
  void doesTurretHaveRoomPositiveAtLimit() {
    var actual = BaseTurretCalculator.doesTurretHaveRoom(170.0, -180.0, 180.0, 10.0, 5.0);
    assertEquals(true, actual);
  }

  @Test
  void doesTurretNotHaveRoomNegative() {
    var actual = BaseTurretCalculator.doesTurretHaveRoom(-178.0, -180.0, 180.0, 10.0, 5.0);
    assertEquals(false, actual);
  }

  @Test
  void doesTurretNotHaveRoomPositive() {
    var actual = BaseTurretCalculator.doesTurretHaveRoom(178.0, -180.0, 180.0, 10.0, 5.0);
    assertEquals(false, actual);
  }

  @Test
  void getOptimalAngleNegative() {
    var minAngle = -270;
    var maxAngle = 270;
    var current = -270;
    var target = 180;
    var actual = BaseTurretCalculator.getOptimalAngle(target, current, minAngle, maxAngle);
    var expected = -180;
    assertEquals(expected, actual, 1e-9);
  }

  @Test
  void getOptimalAngleNegativeEnd() {
    var minAngle = -270;
    var maxAngle = 270;
    var current = 270;
    var target = -90;
    var actual = BaseTurretCalculator.getOptimalAngle(target, current, minAngle, maxAngle);
    var expected = 270;
    assertEquals(expected, actual, 1e-9);
  }

  @Test
  void getOptimalAnglePositive() {
    var minAngle = -270;
    var maxAngle = 270;
    var current = 90;
    var target = 180;
    var actual = BaseTurretCalculator.getOptimalAngle(target, current, minAngle, maxAngle);
    var expected = 180;
    assertEquals(expected, actual, 1e-9);
  }

  @Test
  void getSmartUnwrapAngleNegative() {
    double target = -100;
    double current = 260;
    double minTurretAngle = -270;
    double maxTurretAngle = 270;
    double tolerance = 30;

    var expected = -100;
    var actual =
        BaseTurretCalculator.getSmartUnwrapAngle(
            target, current, minTurretAngle, maxTurretAngle, tolerance);
    assertEquals(expected, actual, 1e-9);
  }

  @Test
  void getSmartUnwrapAngleNegativeNoUnwrapNeeded() {
    double target = -90;
    double current = 50;
    double minTurretAngle = -270;
    double maxTurretAngle = 270;
    double tolerance = 30;

    var expected = -90;
    var actual =
        BaseTurretCalculator.getSmartUnwrapAngle(
            target, current, minTurretAngle, maxTurretAngle, tolerance);
    assertEquals(expected, actual, 1e-9);
  }

  @Test
  void getSmartUnwrapAnglePositive() {
    double target = 110;
    double current = -250;
    double minTurretAngle = -270;
    double maxTurretAngle = 270;
    double tolerance = 30;

    var expected = 110;
    var actual =
        BaseTurretCalculator.getSmartUnwrapAngle(
            target, current, minTurretAngle, maxTurretAngle, tolerance);
    assertEquals(expected, actual, 1e-9);
  }

  @Test
  void getSmartUnwrapAnglePositiveNoUnwrapNeeded() {
    double target = 90;
    double current = 50;
    double minTurretAngle = -270;
    double maxTurretAngle = 270;
    double tolerance = 30;

    var expected = 90;
    var actual =
        BaseTurretCalculator.getSmartUnwrapAngle(
            target, current, minTurretAngle, maxTurretAngle, tolerance);
    assertEquals(expected, actual, 1e-9);
  }

  @Test
  void getSmartUnwrapDifferentThanOptimal() {
    double target = -90;
    double current = 270;
    double minTurretAngle = -270;
    double maxTurretAngle = 270;
    double tolerance = 30;

    var optimalAngle =
        BaseTurretCalculator.getOptimalAngle(target, current, minTurretAngle, maxTurretAngle);
    var smartUnwrapAngle =
        BaseTurretCalculator.getSmartUnwrapAngle(
            target, current, minTurretAngle, maxTurretAngle, tolerance);

    assertNotEquals(optimalAngle, smartUnwrapAngle, 1e-9);
  }
}
