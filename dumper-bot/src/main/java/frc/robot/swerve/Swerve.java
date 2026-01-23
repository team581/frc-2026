package frc.robot.swerve;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.team581.math.MathHelpers;
import com.team581.trailblazer.Trailblazer;
import com.team581.trailblazer.segments.AutoSegment;
import com.team581.util.FieldUtil;
import com.team581.util.FmsUtil;
import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.generated.RobotTunerConstants.TunerSwerveDrivetrain;
import frc.robot.util.scheduling.SubsystemPriority;
import java.util.function.DoubleSupplier;
import org.jspecify.annotations.Nullable;

public class Swerve extends StateMachineSubsystem<SwerveState> {
  public static final double MAX_SPEED = 4.75;

  public static final double MAX_HUB_SPEED = 2.0;

  private static final DoubleSupplier WALL_SNAPS_VELOCITY_ANGLE_THRESHOLD =
      DogLog.tunable("Swerve/WallSnaps/VelocityAngleThresholdDegrees", 90.0, Degrees);

  private static final DoubleSupplier WALL_SNAPS_ROTATION_ANGLE_THRESHOLD =
      DogLog.tunable("Swerve/WallSnaps/RotationAngleThresholdDegrees", 40.0, Degrees);
  private static final DoubleSupplier WALL_SNAPS_DISTANCE_THRESHOLD =
      DogLog.tunable("Swerve/WallSnaps/DistanceThresholdMeters", 1.25);

  private static final double MAX_ANGULAR_RATE = Units.rotationsToRadians(4);
  private static final Rotation2d TELEOP_MAX_ANGULAR_RATE = Rotation2d.fromRotations(2);

  private static final double SIM_LOOP_PERIOD = Units.millisecondsToSeconds(5);

  private static final PhoenixPIDController ORIGINAL_HEADING_PID =
      new PhoenixPIDController(5.75, 0, 0);

  public final TunerSwerveDrivetrain drivetrain;
  private final Trailblazer trailblazer;

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

  private final SwerveRequest.FieldCentricFacingAngle teleopSnapsIntakeRequest =
      new SwerveRequest.FieldCentricFacingAngle()
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
          .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
          .withDeadband(0.07)
          .withRotationalDeadband(0.5)
          .withHeadingPID(
              ORIGINAL_HEADING_PID.getP(), ORIGINAL_HEADING_PID.getI(), ORIGINAL_HEADING_PID.getD())
          .withCenterOfRotation(new Translation2d(0.0, 0.0))
          .withMaxAbsRotationalRate(MAX_ANGULAR_RATE / 2);

  private final SwerveRequest.ApplyFieldSpeeds trailblazerRequest =
      new SwerveRequest.ApplyFieldSpeeds()
          .withDriveRequestType(DriveRequestType.Velocity)
          .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);

  private double lastSimTime;
  private @Nullable Notifier simNotifier = null;

  private SwerveDriveState drivetrainState = new SwerveDriveState();
  private ChassisSpeeds robotRelativeSpeeds = new ChassisSpeeds();
  private ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds();
  private Rotation2d snapAngle = Rotation2d.kZero;
  private double distanceToWall = 0.0;
  private boolean visionOnline = false;

  public Swerve(TunerSwerveDrivetrain drivetrain, Trailblazer trailblazer) {
    super(SubsystemPriority.SWERVE, SwerveState.TELEOP);
    this.drivetrain = drivetrain;
    this.trailblazer = trailblazer;

    if (Utils.isSimulation()) {
      startSimThread();
    }

    drivetrain.setStateStdDevs(new Matrix<>(VecBuilder.fill(0.003, 0.003, 0.002)));
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return robotRelativeSpeeds;
  }

  public ChassisSpeeds getFieldRelativeSpeeds() {
    return fieldRelativeSpeeds;
  }

  @Override
  protected SwerveState getNextState(SwerveState currentState) {
    return switch (currentState) {
      case FEEDING_AUTO -> DriverStation.isTeleop() ? SwerveState.FEEDING_TELEOP : currentState;
      case HUB_AIM_AUTO -> DriverStation.isTeleop() ? SwerveState.HUB_AIM_TELEOP : currentState;
      default -> currentState;
    };
  }

  /**
   * Sets the teleop joystick inputs for the swerve drivetrain.
   *
   * @param translationMagnitude The magnitude [-1, 1] of the translation vector.
   * @param direction The direction of the translation vector from the driver perspective (forward
   *     is positive y, right is positive x).
   * @param rotation The rotation [-1, 1] as a percentage of the maximum angular rate.
   */
  public void setTeleopInputs(double translationMagnitude, Rotation2d direction, double rotation) {
    drivetrain.setOperatorPerspectiveForward(
        FmsUtil.isRedAlliance() ? Rotation2d.k180deg : Rotation2d.kZero);

    var translation = new Translation2d(translationMagnitude, direction);

    var forwardVelocity = translation.getY();
    // For us, negative is left, but WPILib treats positive as left
    var sidewaysVelocity = -translation.getX();

    var maxSpeed = getState() == SwerveState.HUB_AIM_TELEOP ? MAX_HUB_SPEED : MAX_SPEED;

    teleopRequest
        .withVelocityX(forwardVelocity * maxSpeed)
        .withVelocityY(sidewaysVelocity * maxSpeed)
        .withRotationalRate(
            // Robots use CCW+ for rotation, but humans use CW+, so we invert it
            -1.0 * rotation * TELEOP_MAX_ANGULAR_RATE.getRadians());
    teleopSnapsRequest
        .withVelocityX(forwardVelocity * maxSpeed)
        .withVelocityY(sidewaysVelocity * maxSpeed);
    teleopSnapsIntakeRequest
        .withVelocityX(forwardVelocity * maxSpeed)
        .withVelocityY(sidewaysVelocity * maxSpeed);

    sendSwerveRequest();
  }

  @Override
  protected void collectInputs() {
    drivetrainState = drivetrain.getState();
    robotRelativeSpeeds = drivetrainState.Speeds;
    fieldRelativeSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(
            robotRelativeSpeeds, drivetrainState.Pose.getRotation());

    if (getState() == SwerveState.INTAKING) {
      distanceToWall =
          MathHelpers.getClosestPointOnRectanglePerimeter(
                  drivetrainState.Pose.getTranslation(), FieldUtil.FIELD_BOUNDS)
              .getDistance(drivetrainState.Pose.getTranslation());
    }
  }

  private void sendSwerveRequest() {
    switch (getState()) {
      case TELEOP -> drivetrain.setControl(teleopRequest);
      case FEEDING_TELEOP, HUB_AIM_TELEOP -> {
        if (MathUtil.isNear(teleopRequest.RotationalRate, 0, teleopRequest.RotationalDeadband)) {
          drivetrain.setControl(teleopSnapsRequest);
        } else {
          drivetrain.setControl(teleopRequest);
        }
      }
      case FEEDING_AUTO, HUB_AIM_AUTO -> {
        drivetrain.setControl(
            trailblazerRequest.withSpeeds(
                trailblazer.getFieldRelativeSetpoint(
                    drivetrainState.Pose, fieldRelativeSpeeds, snapAngle)));
      }
      case INTAKING -> {
        if (MathUtil.isNear(teleopRequest.RotationalRate, 0, teleopRequest.RotationalDeadband)
            && ableToWallSnap()) {
          DogLog.timestamp("Swerve/WallSnaps/Snapping");
          var closestWallPose =
              MathHelpers.getClosestPointOnRectanglePerimeter(
                  drivetrainState.Pose.getTranslation(), FieldUtil.FIELD_BOUNDS);
          var angleToWall = MathHelpers.getDriveDirection(drivetrainState.Pose, closestWallPose);
          drivetrain.setControl(
              teleopSnapsIntakeRequest
                  .withTargetDirection(angleToWall.plus(Rotation2d.k180deg))
                  .withCenterOfRotation(new Translation2d(distanceToWall, 0.0)));

        } else {
          drivetrain.setControl(teleopRequest);
        }
      }
      case TRAILBLAZER ->
          drivetrain.setControl(
              trailblazerRequest.withSpeeds(
                  trailblazer.getFieldRelativeSetpoint(drivetrainState.Pose, fieldRelativeSpeeds)));
    }
  }

  public void normalDriveRequest() {
    if (DriverStation.isAutonomous()) {
      setStateFromRequest(SwerveState.TRAILBLAZER);
    } else {
      setStateFromRequest(SwerveState.TELEOP);
    }
  }

  public void trailblazerDriveRequest(AutoSegment segment) {
    trailblazer.followSegment(segment);
    setStateFromRequest(SwerveState.TRAILBLAZER);
    sendSwerveRequest();
  }

  public void feedingAimRequest(double snapAngle) {
    this.snapAngle = Rotation2d.fromDegrees(snapAngle);
    teleopSnapsRequest.withTargetDirection(this.snapAngle.rotateBy(Rotation2d.k180deg));

    if (DriverStation.isTeleop()) {
      setStateFromRequest(SwerveState.FEEDING_TELEOP);
      sendSwerveRequest();
    } else {
      setStateFromRequest(SwerveState.FEEDING_AUTO);
      sendSwerveRequest();
    }
  }

  public void hubAimRequest(double snapAngle) {
    this.snapAngle = Rotation2d.fromDegrees(snapAngle);
    teleopSnapsRequest.withTargetDirection(this.snapAngle.rotateBy(Rotation2d.k180deg));

    if (DriverStation.isTeleop()) {
      setStateFromRequest(SwerveState.HUB_AIM_TELEOP);
      sendSwerveRequest();
    } else {
      setStateFromRequest(SwerveState.HUB_AIM_AUTO);
      sendSwerveRequest();
    }
  }

  public void intakeDriveRequest() {
    if (DriverStation.isTeleop()) {
      setStateFromRequest(SwerveState.INTAKING);
    }
  }

  private boolean ableToWallSnap() {
    if (!visionOnline) {
      return false;
    }
    var robotPose = drivetrainState.Pose;
    var closestWallTranslation =
        MathHelpers.getClosestPointOnRectanglePerimeter(
            robotPose.getTranslation(), FieldUtil.FIELD_BOUNDS);

    DogLog.log(
        "Swerve/WallSnaps/ClosestWallPose", new Pose2d(closestWallTranslation, Rotation2d.kZero));

    // Check if velocity angle is toward wall
    var velocityAngle = MathHelpers.getDriveDirection(fieldRelativeSpeeds);
    DogLog.log("Swerve/WallSnaps/VelocityAngle", velocityAngle.getDegrees(), Degrees);

    var angleToWall = MathHelpers.getDriveDirection(robotPose, closestWallTranslation);
    DogLog.log("Swerve/WallSnaps/AngleToWall", angleToWall.getDegrees(), Degrees);

    var velocityAngleTowardWall =
        MathHelpers.getLinearVelocity(fieldRelativeSpeeds) > 0.01
            && MathUtil.isNear(
                angleToWall.getDegrees(),
                velocityAngle.getDegrees(),
                WALL_SNAPS_VELOCITY_ANGLE_THRESHOLD.getAsDouble(),
                -180,
                180);
    var rotationAngleTowardWall =
        MathUtil.isNear(
            angleToWall.getDegrees(),
            robotPose.getRotation().getDegrees(),
            WALL_SNAPS_ROTATION_ANGLE_THRESHOLD.getAsDouble(),
            -180,
            180);

    var distanceToWallThreshold = distanceToWall < WALL_SNAPS_DISTANCE_THRESHOLD.getAsDouble();
    DogLog.log("Swerve/WallSnaps/DistanceToWall", distanceToWallThreshold);
    DogLog.log("Swerve/WallSnaps/VelocityAngleTowardWall", velocityAngleTowardWall);
    DogLog.log("Swerve/WallSnaps/RotationAngleTowardWall", rotationAngleTowardWall);
    return distanceToWallThreshold && velocityAngleTowardWall && rotationAngleTowardWall;
  }

  @Override
  public void whileInState(SwerveState currentState) {
    DogLog.log("Swerve/SnapAngle", snapAngle.getDegrees(), Degrees);
    DogLog.log("Swerve/ModuleStates", drivetrainState.ModuleStates);
    DogLog.log("Swerve/ModuleTargets", drivetrainState.ModuleTargets);
    DogLog.log("Swerve/RobotRelativeSpeeds", drivetrainState.Speeds);
  }

  public void setVisionOnline(boolean online) {
    visionOnline = online;
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
