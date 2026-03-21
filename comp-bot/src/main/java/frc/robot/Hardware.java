package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.DifferentialMotorConstants;
import com.ctre.phoenix6.mechanisms.SimpleDifferentialMechanism;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.deploy.DeployConfig;
import frc.robot.generated.CompTunerConstants;
import frc.robot.generated.CompTunerConstants.TunerSwerveDrivetrain;

public class Hardware {
  public final XboxController driverController = new XboxController(0);
  public final XboxController operatorController = new XboxController(1);

  private final CANBus canivore = new CANBus("581CANivore");
  private final CANBus rio = new CANBus();

  public final TalonFX shooterLeftMotor = new TalonFX(15, rio);
  public final TalonFX shooterRightMotor = new TalonFX(16, rio);
  public final TalonFX shooterMiddleMotor = new TalonFX(40, rio);
  public final TalonFX intakeLeftMotor = new TalonFX(21, rio);
  public final TalonFX intakeRightMotor = new TalonFX(27, rio);

  public final SimpleDifferentialMechanism<TalonFX> deployDifferentialMechanism =
      new SimpleDifferentialMechanism<>(
          TalonFX::new,
          new DifferentialMotorConstants<TalonFXConfiguration>()
              .withCANBusName(canivore.getName())
              .withLeaderId(22)
              .withFollowerId(23)
              .withAlignment(MotorAlignmentValue.Opposed)
              .withLeaderInitialConfigs(DeployConfig.LEFT_MOTOR_CONFIG)
              .withFollowerInitialConfigs(DeployConfig.RIGHT_MOTOR_CONFIG)
              .withFollowerUsesCommonLeaderConfigs(true));
  public final TalonFX shooterHoodMotor = new TalonFX(24, rio);

  public final CANrange hopperCANRange = new CANrange(28, canivore);

  public final TalonFX kickerLeftMotor = new TalonFX(29, canivore);
  public final TalonFX kickerRightMotor = new TalonFX(30, canivore);
  public final TalonFX feederMotor = new TalonFX(31, canivore);
  public final TalonFX conveyorLeftMotor = new TalonFX(32, canivore);
  public final TalonFX conveyorRightMotor = new TalonFX(33, canivore);

  public final TunerSwerveDrivetrain drivetrain =
      new TunerSwerveDrivetrain(
          CompTunerConstants.DrivetrainConstants,
          CompTunerConstants.FrontLeft,
          CompTunerConstants.FrontRight,
          CompTunerConstants.BackLeft,
          CompTunerConstants.BackRight);
}
