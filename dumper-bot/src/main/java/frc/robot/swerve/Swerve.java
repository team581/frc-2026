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
import com.team581.math.PolarChassisSpeeds;
import com.team581.swerve.DriveSource;
import com.team581.swerve.DriveSourceType;
import com.team581.swerve.SwerveAssist;
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
import frc.robot.generated.RobotTunerConstants.TunerSwerveDrivetrain;
import frc.robot.health.HealthManager;
import frc.robot.util.scheduling.SubsystemPriority;
import java.util.function.DoubleSupplier;
import org.jspecify.annotations.Nullable;

public class Swerve extends StateMachineSubsystem<SwerveState> {
  public static final double TRANSLATION_STD_DEV = 0.003;

  public static final double MAX_SPEED = 4.75;

  public static final double MAX_HUB_SPEED = 2.0;
  public static final Rotation2d TELEOP_MAX_ANGULAR_RATE = Rotation2d.fromRotations(2);

  private static final DoubleSupplier WALL_SNAPS_VELOCITY_ANGLE_THRESHOLD =
      DogLog.tunable("Swerve/WallSnaps/VelocityAngleThresholdDegrees", 30.0, Degrees);

  private static final DoubleSupplier WALL_SNAPS_ROTATION_ANGLE_THRESHOLD =
      DogLog.tunable("Swerve/WallSnaps/RotationAngleThresholdDegrees", 40.0, Degrees);
  private static final DoubleSupplier WALL_SNAPS_DISTANCE_THRESHOLD =
      DogLog.tunable("Swerve/WallSnaps/DistanceThresholdMeters", 2.0);
  private static final DoubleSupplier MIN_ROBOT_VELOCITY_FOR_DIRECTION_SNAPS =
      DogLog.tunable("Swerve/MinRobotVelocityForDirectionSnapsMetersPerSecond", 0.5);

  private static final double MAX_ANGULAR_RATE = Units.rotationsToRadians(4);

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

  private final SwerveRequest.FieldCentricFacingAngle driverPerspectiveSnapsOpenLoop =
      new SwerveRequest.FieldCentricFacingAngle()
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
          .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
          .withDeadband(0.07)
          .withRotationalDeadband(0.5)
          .withHeadingPID(
              ORIGINAL_HEADING_PID.getP(), ORIGINAL_HEADING_PID.getI(), ORIGINAL_HEADING_PID.getD())
          .withMaxAbsRotationalRate(MAX_ANGULAR_RATE);

  private final SwerveRequest.FieldCentricFacingAngle drivePerspectiveIntakeSnapsOpenLoop =
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
          .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
          .withHeadingPID(
              ORIGINAL_HEADING_PID.getP(),
              ORIGINAL_HEADING_PID.getI(),
              ORIGINAL_HEADING_PID.getD());

  private final SwerveRequest.FieldCentricFacingAngle fieldCentricIntakeSnapsClosedLoop =
      new SwerveRequest.FieldCentricFacingAngle()
          .withDriveRequestType(DriveRequestType.Velocity)
          .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
          .withHeadingPID(
              ORIGINAL_HEADING_PID.getP(),
              ORIGINAL_HEADING_PID.getI(),
              ORIGINAL_HEADING_PID.getD());

  private final CircularFilter lastDriveDirectionFilter = new CircularFilter(15);

  private final HealthManager health;

  private double lastSimTime;
  private @Nullable Notifier simNotifier = null;

  private SwerveDriveState drivetrainState = new SwerveDriveState();
  private Pose2d robotPose = Pose2d.kZero;
  private ChassisSpeeds robotRelativeSpeeds = new ChassisSpeeds();
  private ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds();
  private Translation2d lastWallIntakePoint = Translation2d.kZero;
  private double distanceToWallIntakePoint = 0.0;
  private Rotation2d filteredLastDriveDirection = Rotation2d.kZero;
  private PolarChassisSpeeds intakeAssistSpeeds = new PolarChassisSpeeds();
  private Rotation2d scoreAngle = Rotation2d.kZero;
  private Rotation2d feedAngle = Rotation2d.kZero;
  private boolean ableToBumpAssist = false;

  public Swerve(TunerSwerveDrivetrain drivetrain, DriveSource driveSource, HealthManager health) {
    super(SubsystemPriority.SWERVE, SwerveState.MANUAL);
    this.drivetrain = drivetrain;
    this.driveSource = driveSource;
    this.health = health;

    if (Utils.isSimulation()) {
      startSimThread();
    }

    drivetrain.setStateStdDevs(
        new Matrix<>(VecBuilder.fill(TRANSLATION_STD_DEV, TRANSLATION_STD_DEV, 0.002)));
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
    robotPose = drivetrainState.Pose;
    robotRelativeSpeeds = drivetrainState.Speeds;
    fieldRelativeSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds, robotPose.getRotation());

    ableToBumpAssist =
        FeatureFlags.BUMP_ASSIST.getAsBoolean()
            && driveSource.getDriveSourceType() == DriveSourceType.DRIVER_PERSPECTIVE_OPEN_LOOP
            && health.isLocalizationHealthy()
            && SwerveAssist.ableToBumpAssist(robotPose, fieldRelativeSpeeds);

    if (getState() == SwerveState.INTAKE) {
      lastWallIntakePoint =
          MathHelpers.getIntersectionOnRectanglePerimeter(
              robotPose.getTranslation(), FieldUtil.FIELD_BOUNDS, filteredLastDriveDirection);
      distanceToWallIntakePoint = lastWallIntakePoint.getDistance(robotPose.getTranslation());

      filteredLastDriveDirection =
          Rotation2d.fromDegrees(
              lastDriveDirectionFilter.calculate(
                  new Translation2d(
                          fieldRelativeSpeeds.vxMetersPerSecond,
                          fieldRelativeSpeeds.vyMetersPerSecond)
                      .getAngle()
                      .plus(Rotation2d.k180deg)
                      .getDegrees()));
    }
  }

  public void setIntakeAssistSpeeds(PolarChassisSpeeds speeds) {
    intakeAssistSpeeds = speeds;
  }

  @Override
  public void whileInState(SwerveState currentState) {
    drivetrain.setOperatorPerspectiveForward(
        FmsUtil.isRedAlliance() ? Rotation2d.k180deg : Rotation2d.kZero);

    switch (currentState) {
      case MANUAL -> {
        var speeds = driveSource.getRequestedSpeeds();

        if (ableToBumpAssist) {
          drivetrain.setControl(
              driverPerspectiveSnapsOpenLoop
                  .withVelocityX(speeds.vxMetersPerSecond)
                  .withVelocityY(speeds.vyMetersPerSecond)
                  .withTargetDirection(
                      SwerveAssist.getRoundedSnapAngle(
                          drivetrainState.Pose.getRotation(), SwerveAssist.BUMP_SNAP_ROUND_ANGLE)));
        } else {
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
      }
      case SCORE -> {
        var speeds = driveSource.getRequestedSpeeds(scoreAngle);

        if (ableToBumpAssist) {
          drivetrain.setControl(
              driverPerspectiveSnapsOpenLoop
                  .withVelocityX(speeds.vxMetersPerSecond)
                  .withVelocityY(speeds.vyMetersPerSecond)
                  .withTargetDirection(
                      SwerveAssist.getRoundedSnapAngle(
                          drivetrainState.Pose.getRotation(), SwerveAssist.BUMP_SNAP_ROUND_ANGLE)));
        } else {
          var swerveRequest =
              driveSource.getDriveSourceType() == DriveSourceType.DRIVER_PERSPECTIVE_OPEN_LOOP
                  ? driverPerspectiveSnapsOpenLoop
                  : fieldCentricSnapsClosedLoop;

          // We want just the translation to be operator perspective, rotation is always blue
          // alliance perspective
          var usedSnapAngle =
              swerveRequest.ForwardPerspective == ForwardPerspectiveValue.BlueAlliance
                  ? scoreAngle
                  : scoreAngle.rotateBy(Rotation2d.k180deg);

          DogLog.log("Swerve/UsedSnapAngle", usedSnapAngle.getDegrees(), Degrees);

          drivetrain.setControl(
              swerveRequest
                  .withVelocityX(speeds.vxMetersPerSecond)
                  .withVelocityY(speeds.vyMetersPerSecond)
                  .withTargetDirection(usedSnapAngle));
        }
      }
      case FEED -> {
        var speeds = driveSource.getRequestedSpeeds(feedAngle);

        if (ableToBumpAssist) {
          drivetrain.setControl(
              driverPerspectiveSnapsOpenLoop
                  .withVelocityX(speeds.vxMetersPerSecond)
                  .withVelocityY(speeds.vyMetersPerSecond)
                  .withTargetDirection(
                      SwerveAssist.getRoundedSnapAngle(
                          drivetrainState.Pose.getRotation(), SwerveAssist.BUMP_SNAP_ROUND_ANGLE)));
        } else {
          var swerveRequest =
              driveSource.getDriveSourceType() == DriveSourceType.DRIVER_PERSPECTIVE_OPEN_LOOP
                  ? driverPerspectiveSnapsOpenLoop
                  : fieldCentricSnapsClosedLoop;

          // We want just the translation to be operator perspective, rotation is always blue
          // alliance perspective
          var usedSnapAngle =
              swerveRequest.ForwardPerspective == ForwardPerspectiveValue.BlueAlliance
                  ? feedAngle
                  : feedAngle.rotateBy(Rotation2d.k180deg);

          drivetrain.setControl(
              swerveRequest
                  .withVelocityX(speeds.vxMetersPerSecond)
                  .withVelocityY(speeds.vyMetersPerSecond)
                  .withTargetDirection(usedSnapAngle));
        }
      }
      case INTAKE -> {
        var speeds = driveSource.getRequestedSpeeds().plus(intakeAssistSpeeds);

        if (ableToBumpAssist) {
          drivetrain.setControl(
              driverPerspectiveSnapsOpenLoop
                  .withVelocityX(speeds.vxMetersPerSecond)
                  .withVelocityY(speeds.vyMetersPerSecond)
                  .withTargetDirection(
                      SwerveAssist.getRoundedSnapAngle(
                          drivetrainState.Pose.getRotation(), SwerveAssist.BUMP_SNAP_ROUND_ANGLE)));
        } else if (ableToWallSnap()) {
          DogLog.timestamp("Swerve/WallSnaps/Snapping");
          var closestWallPose =
              MathHelpers.getClosestPointOnRectanglePerimeter(
                  robotPose.getTranslation(), FieldUtil.FIELD_BOUNDS);
          var angleToWall = MathHelpers.getDriveDirection(robotPose, closestWallPose);
          var centerOfRotationRobotRelative =
              lastWallIntakePoint
                  .minus(robotPose.getTranslation())
                  .rotateBy(robotPose.getRotation().unaryMinus());
          DogLog.log(
              "Swerve/WallSnaps/CenterOfRotation",
              new Pose2d(lastWallIntakePoint, Rotation2d.kZero));

          var swerveSnapsRequest =
              driveSource.getDriveSourceType() == DriveSourceType.DRIVER_PERSPECTIVE_OPEN_LOOP
                  ? drivePerspectiveIntakeSnapsOpenLoop
                  : fieldCentricIntakeSnapsClosedLoop;
          drivetrain.setControl(
              swerveSnapsRequest
                  .withVelocityX(speeds.vxMetersPerSecond)
                  .withVelocityY(speeds.vyMetersPerSecond)
                  .withTargetDirection(angleToWall.plus(Rotation2d.k180deg))
                  .withCenterOfRotation(centerOfRotationRobotRelative));
        } else if (ableToDirectionSnap()) {
          DogLog.timestamp("Swerve/DirectionSnaps/Snapping");
          var swerveSnapsRequest =
              driveSource.getDriveSourceType() == DriveSourceType.DRIVER_PERSPECTIVE_OPEN_LOOP
                  ? drivePerspectiveIntakeSnapsOpenLoop
                  : fieldCentricIntakeSnapsClosedLoop;
          drivetrain.setControl(
              swerveSnapsRequest
                  .withVelocityX(speeds.vxMetersPerSecond)
                  .withVelocityY(speeds.vyMetersPerSecond)
                  .withTargetDirection(filteredLastDriveDirection.plus(Rotation2d.k180deg))
                  .withCenterOfRotation(Translation2d.kZero));
        } else {
          var swerveRequest =
              driveSource.getDriveSourceType() == DriveSourceType.DRIVER_PERSPECTIVE_OPEN_LOOP
                  ? driverPerspectiveOpenLoop
                  : fieldCentricClosedLoop;
          DogLog.timestamp("Swerve/Intake/NoSnapping");
          drivetrain.setControl(
              swerveRequest
                  .withVelocityX(speeds.vxMetersPerSecond)
                  .withVelocityY(speeds.vyMetersPerSecond));
        }
      }
    }

    DogLog.log("Swerve/ScoreAngle", scoreAngle.getDegrees(), Degrees);
    DogLog.log("Swerve/FeedAngle", feedAngle.getDegrees(), Degrees);
    DogLog.log("Swerve/ModuleStates", drivetrainState.ModuleStates);
    DogLog.log("Swerve/ModuleTargets", drivetrainState.ModuleTargets);
    DogLog.log("Swerve/RobotRelativeSpeeds", drivetrainState.Speeds);
    DogLog.log("SwerveAssist/Bump/AbleToBumpAssist", ableToBumpAssist);
  }

  public void normalDriveRequest() {
    setStateFromRequest(SwerveState.MANUAL);
  }

  public void feedDriveRequest(double snapAngle) {
    feedAngle = Rotation2d.fromDegrees(snapAngle);
    setStateFromRequest(SwerveState.FEED);
  }

  public void scoreDriveRequest(double angleToHub) {
    scoreAngle = Rotation2d.fromDegrees(angleToHub);
    setStateFromRequest(SwerveState.SCORE);
  }

  public void intakeDriveRequest() {
    if (DriverStation.isTeleop()) {
      setStateFromRequest(SwerveState.INTAKE);
    }
  }

  private boolean ableToDirectionSnap() {
    if (!FeatureFlags.INTAKE_DIRECTIONAL_SNAPS.getAsBoolean()) {
      return false;
    }

    var robotVelocity = MathHelpers.getLinearVelocity(fieldRelativeSpeeds);
    return robotVelocity > MIN_ROBOT_VELOCITY_FOR_DIRECTION_SNAPS.getAsDouble();
  }

  private boolean ableToWallSnap() {
    if (!health.isLocalizationHealthy() || !FeatureFlags.INTAKE_WALL_SNAPS.getAsBoolean()) {
      return false;
    }
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
