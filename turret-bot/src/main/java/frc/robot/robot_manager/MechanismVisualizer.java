package frc.robot.robot_manager;

import com.team581.GlobalConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/** Visualizes the turret rotation on a top-down view of the robot. */
public final class MechanismVisualizer {
  // Robot frame dimensions (top-down view)
  private static final double ROBOT_WIDTH_METERS = Units.inchesToMeters(28);
  private static final double TURRET_LENGTH_METERS = Units.inchesToMeters(12);

  // Add padding around the robot for visualization
  private static final double CANVAS_SIZE = ROBOT_WIDTH_METERS + Units.inchesToMeters(4);

  private static final Mechanism2d MECHANISM =
      new Mechanism2d(CANVAS_SIZE, CANVAS_SIZE, new Color8Bit(new Color("#121212")));

  // Root at the center of the robot (turret pivot point)
  private static final MechanismRoot2d ROOT =
      MECHANISM.getRoot("turret_pivot", CANVAS_SIZE / 2.0, CANVAS_SIZE / 2.0);

  // Turret ligament pointing in the direction the turret is aiming
  private static final MechanismLigament2d TURRET =
      ROOT.append(
          new MechanismLigament2d(
              "turret", TURRET_LENGTH_METERS, 0, 8, new Color8Bit(Color.kFirstRed)));

  /**
   * Logs the turret visualization to SmartDashboard.
   *
   * @param turretAngleDegrees The current turret angle in degrees.
   */
  public static void log(double turretAngleDegrees) {
    if (!GlobalConfig.IS_DEVELOPMENT) {
      return;
    }

    SmartDashboard.putData("TurretVisualization", MECHANISM);

    TURRET.setAngle(turretAngleDegrees);
  }

  private MechanismVisualizer() {}
}
