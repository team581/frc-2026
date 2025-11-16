package frc.robot.swerve;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.team581.trailblazer.SwerveBase;
import com.team581.util.FmsUtil;
import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.config.RobotConfig;
import frc.robot.generated.RobotTunerConstants.TunerSwerveDrivetrain;
import frc.robot.util.scheduling.SubsystemPriority;
import org.jspecify.annotations.Nullable;

public class SwerveSubsystem extends StateMachineSubsystem<SwerveState> implements SwerveBase {
  public static final double MAX_SPEED = 4.75;

  private static final double MAX_ANGULAR_RATE = Units.rotationsToRadians(4);
  private static final Rotation2d TELEOP_MAX_ANGULAR_RATE = Rotation2d.fromRotations(2);

  private static final double SIM_LOOP_PERIOD = Units.millisecondsToSeconds(5);

  private static final PhoenixPIDController ORIGINAL_HEADING_PID =
      RobotConfig.get().swerve().snapController();

  private final TunerSwerveDrivetrain drivetrain;

  private final SwerveRequest.FieldCentric teleopRequest =
      new SwerveRequest.FieldCentric()
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
          .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
          .withDeadband(0.07)
          .withRotationalDeadband(0.05);

  private final SwerveRequest.FieldCentricFacingAngle teleopSnapsRequest =
      new SwerveRequest.FieldCentricFacingAngle()
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
          .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
          .withDeadband(0.07)
          .withRotationalDeadband(0.5)
          .withHeadingPID(
              ORIGINAL_HEADING_PID.getP(), ORIGINAL_HEADING_PID.getI(), ORIGINAL_HEADING_PID.getD())
          .withMaxAbsRotationalRate(MAX_ANGULAR_RATE);

  private final SwerveRequest.ApplyFieldSpeeds trailblazerRequest =
      new SwerveRequest.ApplyFieldSpeeds()
          .withDriveRequestType(DriveRequestType.Velocity)
          .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);

  private double lastSimTime;
  private @Nullable Notifier simNotifier = null;

  private SwerveDriveState drivetrainState = new SwerveDriveState();
  private ChassisSpeeds robotRelativeSpeeds = new ChassisSpeeds();
  private ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds();

  private double teleopSlowModePercent = 1.0;

  public SwerveSubsystem(TunerSwerveDrivetrain drivetrain) {
    super(SubsystemPriority.SWERVE, SwerveState.TELEOP);
    this.drivetrain = drivetrain;

    if (Utils.isSimulation()) {
      startSimThread();
    }

    drivetrain.setStateStdDevs(new Matrix<>(VecBuilder.fill(0.003, 0.003, 0.002)));
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return robotRelativeSpeeds;
  }

  @Override
  public ChassisSpeeds getFieldRelativeSpeeds() {
    return fieldRelativeSpeeds;
  }

  @Override
  protected SwerveState getNextState(SwerveState currentState) {
    // Ensure that we are in an auto state during auto, and a teleop state during teleop
    if (DriverStation.isAutonomous()) {
      return SwerveState.TRAILBLAZER;
    }

    return currentState;
  }

  /**
   * @param translationMagnitude The magnitude [-1, 1] of the translation vector.
   * @param direction The direction of the translation vector from the driver perspective (forward
   *     is positive y, right is positive x).
   * @param rotation The rotation [-1, 1] as a percentage of the maximum angular rate.
   */
  public void setTeleopInputs(double translationMagnitude, Rotation2d direction, double rotation) {
    var translation = new Translation2d(translationMagnitude, direction);

    var forwardVelocity = translation.getY();
    // For us, negative is left, but WPILib treats positive as left
    var sidewaysVelocity = -translation.getX();

    teleopRequest
        .withVelocityX(forwardVelocity * MAX_SPEED * teleopSlowModePercent)
        .withVelocityY(sidewaysVelocity * MAX_SPEED * teleopSlowModePercent)
        .withRotationalRate(
            rotation * TELEOP_MAX_ANGULAR_RATE.getRadians() * teleopSlowModePercent);
    teleopSnapsRequest
        .withVelocityX(forwardVelocity * MAX_SPEED * teleopSlowModePercent)
        .withVelocityY(sidewaysVelocity * MAX_SPEED * teleopSlowModePercent);

    sendSwerveRequest();
  }

  @Override
  protected void collectInputs() {
    drivetrainState = drivetrain.getState();
    robotRelativeSpeeds = drivetrainState.Speeds;
    fieldRelativeSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(
            robotRelativeSpeeds, drivetrainState.Pose.getRotation());
  }

  private void sendSwerveRequest() {
    switch (getState()) {
      case TELEOP -> drivetrain.setControl(teleopRequest);
      case TELEOP_SNAPS -> {
        if (MathUtil.isNear(teleopRequest.RotationalRate, 0, teleopRequest.RotationalDeadband)) {
          drivetrain.setControl(teleopSnapsRequest);
        } else {
          drivetrain.setControl(teleopRequest);
        }
      }
      case TRAILBLAZER -> drivetrain.setControl(trailblazerRequest);
    }
  }

  public void normalDriveRequest() {
    if (DriverStation.isAutonomous()) {
      setStateFromRequest(SwerveState.TRAILBLAZER);
    } else {
      setStateFromRequest(SwerveState.TELEOP);
    }
  }

  @Override
  public void setFieldRelativeAutoSpeeds(ChassisSpeeds speeds) {
    trailblazerRequest.withSpeeds(speeds);
  }

  public void trailblazerDriveRequest(ChassisSpeeds speeds) {
    trailblazerRequest.withSpeeds(speeds);
    setStateFromRequest(SwerveState.TRAILBLAZER);
  }

  public void snapsDriveRequest(double snapAngle) {
    teleopSnapsRequest.withTargetDirection(Rotation2d.fromDegrees(snapAngle));

    if (DriverStation.isTeleop()) {
      setStateFromRequest(SwerveState.TELEOP_SNAPS);
      sendSwerveRequest();
    }
  }

  @Override
  public void whileInState(SwerveState currentState) {
    drivetrain.setOperatorPerspectiveForward(
        FmsUtil.isRedAlliance() ? Rotation2d.k180deg : Rotation2d.kZero);
    DogLog.log("Swerve/SnapAngle", teleopSnapsRequest.TargetDirection.getDegrees(), Degrees);
    DogLog.log("Swerve/ModuleStates", drivetrainState.ModuleStates);
    DogLog.log("Swerve/ModuleTargets", drivetrainState.ModuleTargets);
    DogLog.log("Swerve/RobotRelativeSpeeds", drivetrainState.Speeds);
  }

  private void startSimThread() {
    lastSimTime = Utils.getCurrentTimeSeconds();

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    simNotifier =
        new Notifier(
            () -> {
              double currentTime = Utils.getCurrentTimeSeconds();
              double deltaTime = currentTime - lastSimTime;
              lastSimTime = currentTime;

              /* use the measured time delta, get battery voltage from WPILib */
              drivetrain.updateSimState(deltaTime, RobotController.getBatteryVoltage());
            });
    simNotifier.startPeriodic(SIM_LOOP_PERIOD);
  }
}
