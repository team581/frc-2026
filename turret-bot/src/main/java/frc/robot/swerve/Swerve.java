package frc.robot.swerve;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.team581.math.CircularFilter;
import com.team581.math.MathHelpers;
import com.team581.swerve.SwerveAssist;
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
import frc.robot.config.FeatureFlags;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.util.scheduling.SubsystemPriority;
import java.util.function.DoubleSupplier;
import org.jspecify.annotations.Nullable;

public class Swerve extends StateMachineSubsystem<SwerveState> {
  public static final double MAX_SPEED = 4.75;

  private static final double MAX_ANGULAR_RATE = Units.rotationsToRadians(0.25);
  private static final double SWERVE_ASSIST_MAX_ANGULAR_RATE = Units.rotationsToRadians(4.0);
  private static final Rotation2d TELEOP_MAX_ANGULAR_RATE = Rotation2d.fromRotations(2);

  private static final DoubleSupplier WALL_SNAPS_VELOCITY_ANGLE_THRESHOLD =
      DogLog.tunable("Swerve/WallSnaps/VelocityAngleThresholdDegrees", 30.0, Degrees);

  private static final DoubleSupplier WALL_SNAPS_ROTATION_ANGLE_THRESHOLD =
      DogLog.tunable("Swerve/WallSnaps/RotationAngleThresholdDegrees", 40.0, Degrees);
  private static final DoubleSupplier WALL_SNAPS_DISTANCE_THRESHOLD =
      DogLog.tunable("Swerve/WallSnaps/DistanceThresholdMeters", 1.75);
  private static final DoubleSupplier MIN_ROBOT_VELOCITY_FOR_DIRECTION_SNAPS =
      DogLog.tunable("Swerve/MinRobotVelocityForDirectionSnapsMetersPerSecond", 0.5);

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
          .withCenterOfRotation(Translation2d.kZero)
          .withMaxAbsRotationalRate(MAX_ANGULAR_RATE);

  // Only for swerve assist because turret bot max teleop angular rate is limited by hardware
  private final SwerveRequest.FieldCentricFacingAngle swerveAssistSnapsRequest =
      new SwerveRequest.FieldCentricFacingAngle()
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
          .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
          .withDeadband(0.07)
          .withRotationalDeadband(0.5)
          .withHeadingPID(
              ORIGINAL_HEADING_PID.getP(), ORIGINAL_HEADING_PID.getI(), ORIGINAL_HEADING_PID.getD())
          .withMaxAbsRotationalRate(SWERVE_ASSIST_MAX_ANGULAR_RATE);

  private final SwerveRequest.ApplyFieldSpeeds trailblazerRequest =
      new SwerveRequest.ApplyFieldSpeeds()
          .withDriveRequestType(DriveRequestType.Velocity)
          .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);

  private double lastSimTime;
  private @Nullable Notifier simNotifier = null;

  private SwerveDriveState drivetrainState = new SwerveDriveState();
  private Pose2d robotPose = Pose2d.kZero;
  private ChassisSpeeds robotRelativeSpeeds = new ChassisSpeeds();
  private ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds();
  private Rotation2d snapAngle = Rotation2d.kZero;

  private boolean ableToTrenchAssist = false;
  private boolean ableToBumpAssist = false;

  private final CircularFilter lastDriveDirectionFilter = new CircularFilter(15);
  private Translation2d lastWallIntakePoint = Translation2d.kZero;
  private double distanceToWallIntakePoint = 0.0;
  private boolean visionOnline = false;
  private Rotation2d filteredLastDriveDirection = Rotation2d.kZero;

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
    swerveAssistSnapsRequest
        .withVelocityX(forwardVelocity * MAX_SPEED)
        .withVelocityY(sidewaysVelocity * MAX_SPEED);
    teleopSnapsIntakeRequest
        .withVelocityX(forwardVelocity * MAX_SPEED)
        .withVelocityY(sidewaysVelocity * MAX_SPEED);

    sendSwerveRequest();

    filteredLastDriveDirection =
        Rotation2d.fromDegrees(
            lastDriveDirectionFilter.calculate(
                new Translation2d(forwardVelocity, sidewaysVelocity)
                    .getAngle()
                    .plus(Rotation2d.k180deg)
                    .getDegrees()));
  }

  @Override
  protected void collectInputs() {
    drivetrainState = drivetrain.getState();
    robotPose = drivetrainState.Pose;
    robotRelativeSpeeds = drivetrainState.Speeds;
    fieldRelativeSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds, robotPose.getRotation());

    ableToTrenchAssist =
        SwerveAssist.ableToTrenchAssist(robotPose, fieldRelativeSpeeds) && visionOnline;
    ableToBumpAssist =
        SwerveAssist.ableToBumpAssist(robotPose, fieldRelativeSpeeds) && visionOnline;

    if (getState() == SwerveState.INTAKING) {
      lastWallIntakePoint =
          MathHelpers.getIntersectionOnRectanglePerimeter(
              drivetrainState.Pose.getTranslation(),
              FieldUtil.FIELD_BOUNDS,
              filteredLastDriveDirection);
      distanceToWallIntakePoint =
          lastWallIntakePoint.getDistance(drivetrainState.Pose.getTranslation());
    }
  }

  // Note: Trench/bump assist has some inverted behavior on blue alliance, fixed on dumper/comp bot
  private void sendSwerveRequest() {
    switch (getState()) {
      case TELEOP -> {
        if (FeatureFlags.TRENCH_ASSIST.getAsBoolean() && ableToTrenchAssist) {
          var trenchAssistSpeeds =
              SwerveAssist.getTrenchAssistSpeeds(
                  robotPose.getTranslation(),
                  new ChassisSpeeds(teleopRequest.VelocityX, teleopRequest.VelocityY, 0.0));
          drivetrain.setControl(
              swerveAssistSnapsRequest
                  .withVelocityX(trenchAssistSpeeds.vxMetersPerSecond)
                  .withVelocityY(trenchAssistSpeeds.vyMetersPerSecond)
                  .withTargetDirection(
                      SwerveAssist.getRoundedSnapAngle(
                          robotPose.getRotation(), SwerveAssist.TRENCH_SNAP_ROUND_ANGLE)));
        } else if (FeatureFlags.BUMP_ASSIST.getAsBoolean() && ableToBumpAssist) {
          drivetrain.setControl(
              swerveAssistSnapsRequest.withTargetDirection(
                  SwerveAssist.getRoundedSnapAngle(
                      drivetrainState.Pose.getRotation(), SwerveAssist.BUMP_SNAP_ROUND_ANGLE)));
        } else {
          drivetrain.setControl(teleopRequest);
        }
      }
      case HUB_AIM_TELEOP -> {
        if (FeatureFlags.TRENCH_ASSIST.getAsBoolean() && ableToTrenchAssist) {
          var trenchAssistSpeeds =
              SwerveAssist.getTrenchAssistSpeeds(robotPose.getTranslation(), fieldRelativeSpeeds);
          drivetrain.setControl(
              swerveAssistSnapsRequest
                  .withVelocityX(trenchAssistSpeeds.vxMetersPerSecond)
                  .withVelocityY(trenchAssistSpeeds.vyMetersPerSecond)
                  .withTargetDirection(
                      SwerveAssist.getRoundedSnapAngle(
                          robotPose.getRotation(), SwerveAssist.TRENCH_SNAP_ROUND_ANGLE)));
        } else if (FeatureFlags.BUMP_ASSIST.getAsBoolean() && ableToBumpAssist) {
          drivetrain.setControl(
              swerveAssistSnapsRequest.withTargetDirection(
                  SwerveAssist.getRoundedSnapAngle(
                      drivetrainState.Pose.getRotation(), SwerveAssist.BUMP_SNAP_ROUND_ANGLE)));
        } else {
          if (MathUtil.isNear(teleopRequest.RotationalRate, 0, teleopRequest.RotationalDeadband)) {
            drivetrain.setControl(teleopSnapsRequest);
          } else {
            drivetrain.setControl(teleopRequest);
          }
        }
      }
      case HUB_AIM_AUTO -> {
        drivetrain.setControl(
            trailblazerRequest.withSpeeds(
                trailblazer.getFieldRelativeSetpoint(robotPose, fieldRelativeSpeeds)));
      }

      case INTAKING -> {
        if (FeatureFlags.TRENCH_ASSIST.getAsBoolean() && ableToTrenchAssist) {
          var trenchAssistSpeeds =
              SwerveAssist.getTrenchAssistSpeeds(robotPose.getTranslation(), fieldRelativeSpeeds);
          drivetrain.setControl(
              swerveAssistSnapsRequest
                  .withVelocityX(trenchAssistSpeeds.vxMetersPerSecond)
                  .withVelocityY(trenchAssistSpeeds.vyMetersPerSecond)
                  .withTargetDirection(
                      SwerveAssist.getRoundedSnapAngle(
                          robotPose.getRotation(), SwerveAssist.TRENCH_SNAP_ROUND_ANGLE)));
        } else if (FeatureFlags.BUMP_ASSIST.getAsBoolean() && ableToBumpAssist) {
          drivetrain.setControl(
              swerveAssistSnapsRequest.withTargetDirection(
                  SwerveAssist.getRoundedSnapAngle(
                      drivetrainState.Pose.getRotation(), SwerveAssist.BUMP_SNAP_ROUND_ANGLE)));
        } else {
          if (MathUtil.isNear(teleopRequest.RotationalRate, 0, teleopRequest.RotationalDeadband)) {
            if (ableToWallSnap()) {
              DogLog.timestamp("Swerve/WallSnaps/Snapping");
              var closestWallPose =
                  MathHelpers.getClosestPointOnRectanglePerimeter(
                      drivetrainState.Pose.getTranslation(), FieldUtil.FIELD_BOUNDS);
              var angleToWall =
                  MathHelpers.getDriveDirection(drivetrainState.Pose, closestWallPose);
              var centerOfRotationRobotRelative =
                  lastWallIntakePoint
                      .minus(drivetrainState.Pose.getTranslation())
                      .rotateBy(drivetrainState.Pose.getRotation().unaryMinus());
              DogLog.log(
                  "Swerve/WallSnaps/CenterOfRotation",
                  new Pose2d(lastWallIntakePoint, Rotation2d.kZero));
              drivetrain.setControl(
                  teleopSnapsIntakeRequest
                      .withTargetDirection(angleToWall.plus(Rotation2d.k180deg))
                      .withCenterOfRotation(centerOfRotationRobotRelative));
            } else if (ableToDirectionSnap()) {
              DogLog.timestamp("Swerve/DirectionSnaps/Snapping");
              drivetrain.setControl(
                  teleopSnapsIntakeRequest
                      .withTargetDirection(filteredLastDriveDirection.plus(Rotation2d.k180deg))
                      .withCenterOfRotation(Translation2d.kZero));
            } else {
              DogLog.timestamp("Swerve/Intake/NoSnapping");
              drivetrain.setControl(teleopRequest);
            }
          } else {
            DogLog.timestamp("Swerve/Intake/NoSnapping");
            drivetrain.setControl(teleopRequest);
          }
        }
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
    trailblazer.setActiveSegment(segment);
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

  public void intakeDriveRequest() {
    if (DriverStation.isTeleop()) {
      setStateFromRequest(SwerveState.INTAKING);
    }
  }

  public boolean snapsNearGoal() {
    var currentRotation = robotPose.getRotation().getDegrees();
    var snapsDirection = teleopSnapsRequest.TargetDirection.getDegrees();
    DogLog.log("Swerve/SnapsAngle", snapsDirection);
    DogLog.log("Swerve/CurrentRotation", currentRotation);

    return MathUtil.isNear(snapsDirection, currentRotation, 5.0, -180.0, 180.0);
  }

  public void setVisionOnline(boolean online) {
    visionOnline = online;
  }

  private boolean ableToDirectionSnap() {
    if (!FeatureFlags.INTAKE_DIRECTIONAL_SNAPS.getAsBoolean()) {
      return false;
    }

    var robotVelocity = MathHelpers.getLinearVelocity(fieldRelativeSpeeds);
    return robotVelocity > MIN_ROBOT_VELOCITY_FOR_DIRECTION_SNAPS.getAsDouble();
  }

  private boolean ableToWallSnap() {
    if (!visionOnline || !FeatureFlags.INTAKE_WALL_SNAPS.getAsBoolean()) {
      return false;
    }
    var robotPose = drivetrainState.Pose;
    var closestWallTranslation =
        MathHelpers.getClosestPointOnRectanglePerimeter(
            robotPose.getTranslation(), FieldUtil.FIELD_BOUNDS);

    DogLog.log(
        "Swerve/WallSnaps/ClosestWallPose", new Pose2d(closestWallTranslation, Rotation2d.kZero));

    // Check if velocity angle is toward wall
    var filteredVelocityAngle = filteredLastDriveDirection;
    DogLog.log(
        "Swerve/WallSnaps/FilteredVelocityAngle", filteredVelocityAngle.getDegrees(), Degrees);

    var angleToWall = MathHelpers.getDriveDirection(robotPose, closestWallTranslation);
    DogLog.log("Swerve/WallSnaps/AngleToWall", angleToWall.getDegrees(), Degrees);

    double robotAngle = robotPose.getRotation().getDegrees();

    var intakeAngleDifference = MathHelpers.angleModulus(angleToWall.getDegrees() - robotAngle);
    DogLog.log("Swerve/WallSnaps/IntakeAngleDifference", intakeAngleDifference);
    var driveAngleDifference =
        MathHelpers.angleModulus(angleToWall.getDegrees() - filteredVelocityAngle.getDegrees());
    DogLog.log("Swerve/WallSnaps/DriveAngleDifference", driveAngleDifference);

    var signMisMatch =
        Math.abs(
                    MathHelpers.angleModulus(
                        filteredVelocityAngle.getDegrees() - angleToWall.getDegrees()))
                > 1e-5
            && Math.signum(intakeAngleDifference) != Math.signum(driveAngleDifference);
    DogLog.log("Swerve/WallSnaps/SignMisMatch", signMisMatch);
    if (signMisMatch) {
      return false;
    }

    var velocityAngleTowardWall =
        MathHelpers.getLinearVelocity(fieldRelativeSpeeds) > 0.01
            && MathUtil.isNear(
                angleToWall.getDegrees(),
                filteredVelocityAngle.getDegrees(),
                WALL_SNAPS_VELOCITY_ANGLE_THRESHOLD.getAsDouble(),
                -180,
                180);
    var rotationAngleTowardWall =
        MathUtil.isNear(
            angleToWall.getDegrees(),
            robotAngle,
            WALL_SNAPS_ROTATION_ANGLE_THRESHOLD.getAsDouble(),
            -180,
            180);

    var distanceToWallThreshold =
        distanceToWallIntakePoint < WALL_SNAPS_DISTANCE_THRESHOLD.getAsDouble();
    DogLog.log("Swerve/WallSnaps/DistanceToWall", distanceToWallThreshold);
    DogLog.log("Swerve/WallSnaps/VelocityAngleTowardWall", velocityAngleTowardWall);
    DogLog.log("Swerve/WallSnaps/RotationAngleTowardWall", rotationAngleTowardWall);
    return distanceToWallThreshold && velocityAngleTowardWall && rotationAngleTowardWall;
  }

  @Override
  public void whileInState(SwerveState currentState) {
    DogLog.log("Swerve/SnapsNearGoal", snapsNearGoal());
    DogLog.log("Swerve/SnapAngle", snapAngle.getDegrees(), Degrees);
    DogLog.log("Swerve/ModuleStates", drivetrainState.ModuleStates);
    DogLog.log("Swerve/ModuleTargets", drivetrainState.ModuleTargets);
    DogLog.log("Swerve/RobotRelativeSpeeds", drivetrainState.Speeds);
    DogLog.log("SwerveAssist/Trench/AbleToTrenchAssist", ableToTrenchAssist);
    DogLog.log("SwerveAssist/Bump/AbleToBumpAssist", ableToBumpAssist);
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
