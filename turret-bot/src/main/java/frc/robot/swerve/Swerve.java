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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.config.FeatureFlags;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.util.scheduling.SubsystemPriority;
import org.jspecify.annotations.Nullable;

public class Swerve extends StateMachineSubsystem<SwerveState> {
  public static final double MAX_SPEED = 4.75;

  private static final double MAX_ANGULAR_RATE = Units.rotationsToRadians(0.25);
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

  private final SwerveRequest.ApplyFieldSpeeds trailblazerRequest =
      new SwerveRequest.ApplyFieldSpeeds()
          .withDriveRequestType(DriveRequestType.Velocity)
          .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);

  private double lastSimTime;
  private @Nullable Notifier simNotifier = null;

  private SwerveDriveState drivetrainState = new SwerveDriveState();
  private Pose2d robotPose = new Pose2d();
  private ChassisSpeeds robotRelativeSpeeds = new ChassisSpeeds();
  private ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds();
  private Rotation2d snapAngle = Rotation2d.kZero;

  // Trench Assist variables
  private final DoubleSubscriber TRENCH_ASSIST_VELOCITY_THRESHOLD =
      DogLog.tunable("Swerve/TrenchAssistVelocityThreshold", 2.0);
  private final DoubleSubscriber TRENCH_ASSIST_ANGLE_THRESHOLD =
      DogLog.tunable("Swerve/TrenchAssistAngleThreshold", 60.0);
  private final double TRENCH_ASSIST_Y_TOLERANCE = Units.inchesToMeters(0.5);
  private final PIDController trenchPidController = new PIDController(15, 0, 0);
  private double TRENCH_ASSIST_Y = 0.0;

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

    teleopRequest
        .withVelocityX(forwardVelocity * MAX_SPEED)
        .withVelocityY(sidewaysVelocity * MAX_SPEED)
        .withRotationalRate(
            // Robots use CCW+ for rotation, but humans use CW+, so we invert it
            -1.0 * rotation * TELEOP_MAX_ANGULAR_RATE.getRadians());
    teleopSnapsRequest
        .withVelocityX(forwardVelocity * MAX_SPEED)
        .withVelocityY(sidewaysVelocity * MAX_SPEED);

    sendSwerveRequest();
  }

  @Override
  protected void collectInputs() {
    drivetrainState = drivetrain.getState();
    robotPose = drivetrainState.Pose;
    robotRelativeSpeeds = drivetrainState.Speeds;
    fieldRelativeSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds, robotPose.getRotation());

    if (robotPose.getY() < FieldUtil.FIELD_WIDTH / 2.0) {
      TRENCH_ASSIST_Y = FieldUtil.BOTTOM_TRENCH_Y;
    } else {
      TRENCH_ASSIST_Y = FieldUtil.TOP_TRENCH_Y;
    }
  }

  private void sendSwerveRequest() {
    switch (getState()) {
      case TELEOP -> {
        if (FeatureFlags.TRENCH_ASSIST.getAsBoolean() && ableToTrenchAssist()) {
          var trenchAssistVelocity = getTrenchAssistVelocity();
          var trenchSnapAngle = Math.round(robotPose.getRotation().getDegrees() / 90.0) * 90.0;
          DogLog.log("Swerve/TrenchAssist/assistVelocity", trenchAssistVelocity);
          DogLog.log("Swerve/TrenchAssist/SnapAngle", trenchSnapAngle);

          if (Math.abs(TRENCH_ASSIST_Y - robotPose.getY()) > TRENCH_ASSIST_Y_TOLERANCE)
            DogLog.timestamp("Swerve/TrenchAssist/ApplyingTrenchAssist");
          DogLog.log("Swerve/TrenchAssist/TrenchAssistY", TRENCH_ASSIST_Y);
          drivetrain.setControl(
              teleopSnapsRequest
                  .withVelocityY(trenchAssistVelocity)
                  .withTargetDirection(
                      Rotation2d.fromDegrees(trenchSnapAngle).rotateBy(Rotation2d.k180deg)));
        } else {
          drivetrain.setControl(teleopRequest);
        }
      }
      case HUB_AIM_TELEOP -> {
        if (MathUtil.isNear(teleopRequest.RotationalRate, 0, teleopRequest.RotationalDeadband)) {
          drivetrain.setControl(teleopSnapsRequest);
        } else {
          drivetrain.setControl(teleopRequest);
        }
      }
      case HUB_AIM_AUTO -> {
        drivetrain.setControl(
            trailblazerRequest.withSpeeds(
                trailblazer.getFieldRelativeSetpoint(robotPose, fieldRelativeSpeeds, snapAngle)));
      }
      case TRAILBLAZER ->
          drivetrain.setControl(
              trailblazerRequest.withSpeeds(
                  trailblazer.getFieldRelativeSetpoint(robotPose, fieldRelativeSpeeds)));
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

  public boolean snapsNearGoal() {
    var currentRotation = robotPose.getRotation().getDegrees();
    var snapsDirection = teleopSnapsRequest.TargetDirection.getDegrees();
    DogLog.log("Swerve/SnapsAngle", snapsDirection);
    DogLog.log("Swerve/CurrentRotation", currentRotation);

    return MathUtil.isNear(snapsDirection, currentRotation, 5.0, -180.0, 180.0);
  }

  private double getTrenchAssistVelocity() {
    return -trenchPidController.calculate(robotPose.getY(), TRENCH_ASSIST_Y);
  }

  private boolean ableToTrenchAssist() {
    var maybeClosestTrenchAssistZone =
        FieldUtil.getCurrentTrenchAssistZone(robotPose.getTranslation());

    // Check if in trench assist zone
    if (maybeClosestTrenchAssistZone.isEmpty()) {
      return false;
    }

    var closestTrenchAssistZone = maybeClosestTrenchAssistZone.orElseThrow();
    DogLog.log("Swerve/TrenchAssist/InZone", closestTrenchAssistZone.getCenter());
    DogLog.log(
        "Swerve/TrenchAssist/ClosestTrench",
        new Pose2d(FieldUtil.getClosestTrenchCenter(robotPose.getTranslation()), Rotation2d.kZero));

    // Check if velocity meets threshold
    if (MathHelpers.getLinearVelocity(fieldRelativeSpeeds)
        <= TRENCH_ASSIST_VELOCITY_THRESHOLD.get()) {
      DogLog.log("Swerve/TrenchAssist/VelocityThreshold", false);
      return false;
    }
    DogLog.log("Swerve/TrenchAssist/VelocityThreshold", true);

    // Check if angle is toward trench
    var velocityAngle = MathHelpers.getDriveDirection(fieldRelativeSpeeds);
    var angleToTrench =
        MathHelpers.getDriveDirection(
            robotPose, FieldUtil.getClosestTrenchCenter(robotPose.getTranslation()));
    DogLog.log(
        "Swerve/TrenchAssist/AngleThreshold",
        MathUtil.isNear(
            velocityAngle.getDegrees(),
            angleToTrench.getDegrees(),
            TRENCH_ASSIST_ANGLE_THRESHOLD.get()));
    return MathUtil.isNear(
        velocityAngle.getDegrees(),
        angleToTrench.getDegrees(),
        TRENCH_ASSIST_ANGLE_THRESHOLD.get());
  }

  @Override
  public void whileInState(SwerveState currentState) {
    DogLog.log("Swerve/SnapsNearGoal", snapsNearGoal());
    DogLog.log("Swerve/SnapAngle", snapAngle.getDegrees(), Degrees);
    DogLog.log("Swerve/ModuleStates", drivetrainState.ModuleStates);
    DogLog.log("Swerve/ModuleTargets", drivetrainState.ModuleTargets);
    DogLog.log("Swerve/RobotRelativeSpeeds", drivetrainState.Speeds);
    DogLog.log("Swerve/TrenchAssist/AbleToTrenchAssist", ableToTrenchAssist());
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
