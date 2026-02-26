package frc.robot.robot_manager;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.shooter_hood.ShooterHoodConfig;

public final class MechanismVisualizer {
  /**
   * If (0, 0, 0) is the robot origin, this translation defines the point that the shooter hood
   * pivots around.
   */
  private static final Translation3d SHOOTER_HOOD_PIVOT_POINT =
      new Translation3d(0.118364, 0, 0.436753);

  /** Distance from the pivot point of the shooter hood to the edge of the hood. */
  private static final double SHOOTER_HOOD_RADIUS = Units.inchesToMeters(6.475600);

  /** Height from the turret to the pivot point of the shooter hood. */
  private static final double SHOOTER_HOOD_HEIGHT_METERS = Units.inchesToMeters(2.867553);

  /** Height from the floor to the bottom of the climber. */
  private static final double CLIMBER_HEIGHT_METERS = Units.inchesToMeters(0.941267);

  /** Angle from the horizontal to the deploy extension point. */
  private static final double DEPLOY_ANGLE_FROM_HORIZONTAL = 15.327113;

  /** Height from the floor to the bottom of the deploy extension point. */
  private static final double DEPLOY_HEIGHT_METERS = Units.inchesToMeters(11.559979);

  private static final Translation2d MECHANISM_AREA = new Translation2d(2, 2);

  private static final double MECHANISM_CENTER_X = MECHANISM_AREA.getX() / 2.0;

  private static final Mechanism2d MECHANISM =
      new Mechanism2d(
          MECHANISM_AREA.getX(), MECHANISM_AREA.getY(), new Color8Bit(new Color("#121212")));

  /** Separate mechanism for the shooter hood, which rotates with the turret. */
  private static final Translation2d SHOOTER_HOOD_MECHANISM_AREA = new Translation2d(1, 1);

  private static final double SHOOTER_HOOD_MECHANISM_CENTER_X =
      SHOOTER_HOOD_MECHANISM_AREA.getX() / 2.0;

  private static final Mechanism2d SHOOTER_HOOD_MECHANISM =
      new Mechanism2d(
          SHOOTER_HOOD_MECHANISM_AREA.getX(),
          SHOOTER_HOOD_MECHANISM_AREA.getY(),
          new Color8Bit(new Color("#121212")));

  private static final MechanismRoot2d CLIMBER =
      MECHANISM.getRoot(
          "climberRoot",
          MECHANISM_CENTER_X + Units.inchesToMeters(12.307293),
          CLIMBER_HEIGHT_METERS);

  private static final MechanismLigament2d CLIMBER_ELEVATOR =
      CLIMBER.append(
          new MechanismLigament2d("climber", 0, 90, 10, new Color8Bit(Color.kFirstBlue)));

  private static final MechanismRoot2d DEPLOY =
      MECHANISM.getRoot(
          "deployRoot", MECHANISM_CENTER_X - Units.inchesToMeters(4.654702), DEPLOY_HEIGHT_METERS);

  private static final MechanismLigament2d DEPLOY_EXTENSION =
      DEPLOY.append(
          new MechanismLigament2d(
              "deploy", 0, DEPLOY_ANGLE_FROM_HORIZONTAL, 10, new Color8Bit(Color.kOrange)));

  private static final MechanismRoot2d SHOOTER_HOOD =
      SHOOTER_HOOD_MECHANISM.getRoot(
          "shooterHoodRoot",
          SHOOTER_HOOD_MECHANISM_CENTER_X - Units.inchesToMeters(4.686288),
          SHOOTER_HOOD_HEIGHT_METERS);

  static {
    SHOOTER_HOOD.append(
        new MechanismLigament2d(
            "shooterHoodBase", SHOOTER_HOOD_RADIUS, 0, 10, new Color8Bit(Color.kFirstRed)));
  }

  private static final MechanismLigament2d SHOOTER_HOOD_PIVOT =
      SHOOTER_HOOD.append(
          new MechanismLigament2d(
              "shooterHood", SHOOTER_HOOD_RADIUS, 0, 10, new Color8Bit(Color.kFirstRed)));

  public static void log(
      Pose2d robotPose,
      double turretAngleDegrees,
      double shooterHoodAngleDegrees,
      double deployLengthInches,
      double climberHeightInches,
      double dyeRotorAngleDegrees) {
    SmartDashboard.putData("Visualization/Static", MECHANISM);
    SmartDashboard.putData("Visualization/Turret", SHOOTER_HOOD_MECHANISM);

    SHOOTER_HOOD_PIVOT.setAngle(shooterHoodAngleDegrees);
    CLIMBER_ELEVATOR.setLength(Units.inchesToMeters(climberHeightInches));
    DEPLOY_EXTENSION.setLength(Units.inchesToMeters(deployLengthInches));

    var turretPose =
        new Pose3d(Translation3d.kZero, new Rotation3d(Rotation2d.fromDegrees(turretAngleDegrees)));
    var shooterHoodPose =
        Pose3d.kZero
            .rotateAround(
                SHOOTER_HOOD_PIVOT_POINT,
                new Rotation3d(
                    0,
                    Math.toRadians(
                        shooterHoodAngleDegrees - ShooterHoodConfig.ANGLE_FROM_HORIZONTAL),
                    0))
            .rotateBy(turretPose.getRotation());
    var deployPose =
        new Pose3d(
            new Translation3d(Units.inchesToMeters(deployLengthInches), 0, 0)
                .rotateBy(new Rotation3d(0, Math.toRadians(DEPLOY_ANGLE_FROM_HORIZONTAL), 0)),
            Rotation3d.kZero);
    var climberPose =
        new Pose3d(
            new Translation3d(0, 0, Units.inchesToMeters(climberHeightInches)), Rotation3d.kZero);
    var dyeRotorPose =
        new Pose3d(
            new Translation3d(0, 0, 0),
            new Rotation3d(Rotation2d.fromDegrees(-dyeRotorAngleDegrees)));

    DogLog.log(
        "SuperstructureVisualization/Components",
        new Pose3d[] {turretPose, shooterHoodPose, deployPose, climberPose, dyeRotorPose});
  }

  private MechanismVisualizer() {}
}
