package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.config.RobotKind;
import frc.robot.generated.CompTunerConstants;
import frc.robot.generated.CompTunerConstants.TunerSwerveDrivetrain;
import frc.robot.generated.PracticeTunerConstants;

public class Hardware {
  public final XboxController driverController = new XboxController(0);
  public final XboxController operatorController = new XboxController(1);

  private final CANBus canivore = new CANBus("581CANivore");
  private final CANBus rio = new CANBus();

  public final TalonFX shooterLeftMotor = new TalonFX(15, rio);
  public final TalonFX shooterRightMotor = new TalonFX(16, rio);
  public final TalonFX turretMotor = new TalonFX(17, canivore);
  public final TalonFX rotorMotor = new TalonFX(18, canivore);
  public final TalonFX horizontalMotor = new TalonFX(19, rio);
  public final TalonFX verticalMotor = new TalonFX(20, rio);
  public final TalonFX intakeMotor = new TalonFX(21, rio);
  public final TalonFX leftDeployMotor = new TalonFX(22, canivore);
  public final TalonFX rightDeployMotor = new TalonFX(23, canivore);
  public final TalonFX shooterHoodMotor = new TalonFX(24, rio);
  public final TalonFX climbMotor = new TalonFX(25, canivore);

  public final CANcoder turretEncoder = new CANcoder(26, canivore);

  public final CANdle candle = new CANdle(27, rio);

  public final CANrange hopperCANRange = new CANrange(28, canivore);

  public final TunerSwerveDrivetrain drivetrain =
      RobotKind.IS_COMP_BOT
          ? new TunerSwerveDrivetrain(
              CompTunerConstants.DrivetrainConstants,
              CompTunerConstants.FrontLeft,
              CompTunerConstants.FrontRight,
              CompTunerConstants.BackLeft,
              CompTunerConstants.BackRight)
          : new TunerSwerveDrivetrain(
              PracticeTunerConstants.DrivetrainConstants,
              PracticeTunerConstants.FrontLeft,
              PracticeTunerConstants.FrontRight,
              PracticeTunerConstants.BackLeft,
              PracticeTunerConstants.BackRight);
}
