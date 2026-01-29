package frc.robot.swerve;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.team581.swerve.DriveSource;
import com.team581.swerve.DriveSourceType;
import com.team581.util.FmsUtil;
import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.generated.RobotTunerConstants.TunerSwerveDrivetrain;
import frc.robot.util.scheduling.SubsystemPriority;
import org.jspecify.annotations.Nullable;

public class Swerve extends StateMachineSubsystem<SwerveState> {
  public static final double MAX_SPEED = 4.75;

  private static final double MAX_ANGULAR_RATE = Units.rotationsToRadians(4);
  public static final Rotation2d TELEOP_MAX_ANGULAR_RATE = Rotation2d.fromRotations(2);

  private static final double SIM_LOOP_PERIOD = Units.millisecondsToSeconds(5);

  private static final PhoenixPIDController ORIGINAL_HEADING_PID =
      new PhoenixPIDController(5.75, 0, 0);

  public final TunerSwerveDrivetrain drivetrain;

  private DriveSource driveSource;

  private final SwerveRequest.FieldCentric driverPerspectiveOpenLoop =
      new SwerveRequest.FieldCentric()
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
          .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
          .withDeadband(0.07)
          .withRotationalDeadband(0.05);

  private final SwerveRequest.FieldCentricFacingAngle drivePerspectiveSnapsOpenLoop =
      new SwerveRequest.FieldCentricFacingAngle()
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
          .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
          .withDeadband(0.07)
          .withRotationalDeadband(0.5)
          .withHeadingPID(
              ORIGINAL_HEADING_PID.getP(), ORIGINAL_HEADING_PID.getI(), ORIGINAL_HEADING_PID.getD())
          .withMaxAbsRotationalRate(MAX_ANGULAR_RATE);

  private final SwerveRequest.FieldCentric fieldCentricClosedLoop =
      new SwerveRequest.FieldCentric()
          .withDriveRequestType(DriveRequestType.Velocity)
          .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);

  private final SwerveRequest.FieldCentricFacingAngle fieldCentricSnapsClosedLoop =
      new SwerveRequest.FieldCentricFacingAngle()
          .withDriveRequestType(DriveRequestType.Velocity)
          .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);

  private double lastSimTime;
  private @Nullable Notifier simNotifier = null;

  private SwerveDriveState drivetrainState = new SwerveDriveState();
  private ChassisSpeeds robotRelativeSpeeds = new ChassisSpeeds();
  private ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds();
  private Rotation2d hubAimAngle = Rotation2d.kZero;

  public Swerve(TunerSwerveDrivetrain drivetrain, DriveSource driveSource) {
    super(SubsystemPriority.SWERVE, SwerveState.MANUAL);
    this.drivetrain = drivetrain;
    this.driveSource = driveSource;

    if (Utils.isSimulation()) {
      startSimThread();
    }

    drivetrain.setStateStdDevs(new Matrix<>(VecBuilder.fill(0.003, 0.003, 0.002)));
  }

  public void setDriveSource(DriveSource driveSource) {
    this.driveSource = driveSource;
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return robotRelativeSpeeds;
  }

  public ChassisSpeeds getFieldRelativeSpeeds() {
    return fieldRelativeSpeeds;
  }

  @Override
  protected void collectInputs() {
    drivetrainState = drivetrain.getState();
    robotRelativeSpeeds = drivetrainState.Speeds;
    fieldRelativeSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(
            robotRelativeSpeeds, drivetrainState.Pose.getRotation());
  }

  public void normalDriveRequest() {
    setStateFromRequest(SwerveState.MANUAL);
  }

  public void hubAimRequest(double angleToHub) {
    hubAimAngle = Rotation2d.fromDegrees(angleToHub);
    setStateFromRequest(SwerveState.HUB_AIM);
  }

  @Override
  public void whileInState(SwerveState currentState) {
    drivetrain.setOperatorPerspectiveForward(
        FmsUtil.isRedAlliance() ? Rotation2d.k180deg : Rotation2d.kZero);

    switch (currentState) {
      case MANUAL -> {
        var speeds = driveSource.getRequestedSpeeds();
        var swerveRequest =
            driveSource.getDriveSourceType() == DriveSourceType.DRIVER_PERSPECTIVE_OPEN_LOOP
                ? driverPerspectiveOpenLoop
                : fieldCentricClosedLoop;

        drivetrain.setControl(
            swerveRequest
                .withVelocityX(speeds.vxMetersPerSecond)
                .withVelocityY(speeds.vyMetersPerSecond)
                .withRotationalRate(speeds.omegaRadiansPerSecond));
      }
      case HUB_AIM -> {
        var speeds = driveSource.getRequestedSpeeds();
        var swerveRequest =
            driveSource.getDriveSourceType() == DriveSourceType.DRIVER_PERSPECTIVE_OPEN_LOOP
                ? drivePerspectiveSnapsOpenLoop
                : fieldCentricSnapsClosedLoop;

        // We want just the translation to be operator perspective, rotation is always blue alliance
        // perspective
        var usedSnapAngle =
            swerveRequest.ForwardPerspective == ForwardPerspectiveValue.BlueAlliance
                ? hubAimAngle
                : hubAimAngle.rotateBy(Rotation2d.k180deg);

        drivetrain.setControl(
            swerveRequest
                .withVelocityX(speeds.vxMetersPerSecond)
                .withVelocityY(speeds.vyMetersPerSecond)
                .withTargetDirection(usedSnapAngle));
      }
    }

    DogLog.log("Swerve/HubAimAngle", hubAimAngle.getDegrees(), Degrees);
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
