package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.DifferentialMechanism;
import com.ctre.phoenix6.mechanisms.DifferentialMotorConstants;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.deploy.DeployConfig;
import frc.robot.generated.CompTunerConstants;
import frc.robot.generated.CompTunerConstants.TunerSwerveDrivetrain;

public class Hardware {
  public final XboxController driverController = new XboxController(0);
  public final XboxController operatorController = new XboxController(1);

  private final CANBus canivore = new CANBus("581CANivore");
  private final CANBus rio = new CANBus();

  public final DifferentialMechanism<TalonFX> deployDifferentialMechanism =
      new DifferentialMechanism<>(
          TalonFX::new,
          new DifferentialMotorConstants<TalonFXConfiguration>()
              .withCANBusName(canivore.getName())
              .withLeaderId(15)
              .withFollowerId(30)
              .withAlignment(MotorAlignmentValue.Opposed)
              .withLeaderInitialConfigs(DeployConfig.LEFT_MOTOR_CONFIG)
              .withFollowerInitialConfigs(DeployConfig.RIGHT_MOTOR_CONFIG)
              .withFollowerUsesCommonLeaderConfigs(true));

  public final TalonFX intakeLeftMotor = new TalonFX(16, rio);
  public final TalonFX intakeRightMotor = new TalonFX(17, rio);

  public final TalonFX conveyorTopMotor = new TalonFX(18, canivore);
  public final TalonFX conveyorBottomMotor = new TalonFX(19, canivore);

  public final TalonFX feederTopMotor = new TalonFX(20, canivore);
  public final TalonFX feederBottomMotor = new TalonFX(21, canivore);

  public final TalonFX shooterHoodMotor = new TalonFX(22, canivore);

  public final TalonFX shooterBottomLeftMotor = new TalonFX(23, canivore);
  public final TalonFX shooterBottomRightMotor = new TalonFX(24, canivore);
  public final TalonFX shooterTopLeftMotor = new TalonFX(25, canivore);
  public final TalonFX shooterTopRightMotor = new TalonFX(26, canivore);

  public final CANrange hopperCANRange = new CANrange(27, canivore);
  public final DigitalInput towerSensor = new DigitalInput(9);
  // public final TalonFX extender = new TalonFX(28);

  public final TunerSwerveDrivetrain drivetrain =
      new TunerSwerveDrivetrain(
          CompTunerConstants.DrivetrainConstants,
          CompTunerConstants.FrontLeft,
          CompTunerConstants.FrontRight,
          CompTunerConstants.BackLeft,
          CompTunerConstants.BackRight);
}
