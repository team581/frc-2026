package frc.robot.swerve;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.team581.math.CircularFilter;
import com.team581.math.MathHelpers;
import com.team581.swerve.DriveSource;
import com.team581.swerve.DriveSourceType;
import com.team581.swerve.SwerveAssist;
import com.team581.swerve.TrailblazerDriveSource;
import com.team581.swerve.XboxControllerDriveSource;
import com.team581.trailblazer.Trailblazer;
import com.team581.util.FieldUtil;
import com.team581.util.FmsUtil;
import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.config.DSOptions;
import frc.robot.config.FeatureFlags;
import frc.robot.generated.CompTunerConstants.TunerSwerveDrivetrain;
import frc.robot.health.HealthManager;
import frc.robot.util.scheduling.SubsystemPriority;
import org.jspecify.annotations.Nullable;

@SuppressWarnings("unused")
public class Swerve extends StateMachineSubsystem<SwerveState> {
  public static final double MAX_SPEED = 4.75;

  public static final double TRANSLATION_STD_DEV = 0.01;

  public static final double MAX_LINEAR_RATE = 4.75;
  private static final int MAX_LINEAR_RATE_SHOOTING = 7;

  private static final double MAX_ANGULAR_RATE = Units.rotationsToRadians(4);
  private static final double MAX_ANGULAR_RATE_SHOOTING = Units.rotationsToRadians(0.5);
  public static final Rotation2d TELEOP_MAX_ANGULAR_RATE = Rotation2d.fromRotations(2);

  private static final double SIM_LOOP_PERIOD = Units.millisecondsToSeconds(5);

  private final SlewRateLimiter scoringXLinearVelocitySlewRateLimiter = new SlewRateLimiter(MAX_LINEAR_RATE_SHOOTING);
  private final SlewRateLimiter scoringYLinearVelocitySlewRateLimiter = new SlewRateLimiter(MAX_LINEAR_RATE_SHOOTING);

  private final CircularFilter lastDriveDirectionFilter = new CircularFilter(15);

  private static final PhoenixPIDController ORIGINAL_HEADING_PID =
      new PhoenixPIDController(5.75, 0, 0);

  public final TunerSwerveDrivetrain drivetrain;

  private final XboxControllerDriveSource teleopDriveSource;
  private final TrailblazerDriveSource trailblazerDriveSource;
  private DriveSource driveSource;

  /** A {@link SwerveRequest} for use with {@link DriveSourceType#DRIVER_PERSPECTIVE_OPEN_LOOP}. */
  private final SwerveRequest.FieldCentric driverPerspectiveOpenLoop =
      new SwerveRequest.FieldCentric()
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
          .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
          .withDeadband(0.07)
          .withRotationalDeadband(0.05);

  /**
   * A {@link SwerveRequest} for use with {@link DriveSourceType#DRIVER_PERSPECTIVE_OPEN_LOOP}, but
   * overrides the angular velocity to instead snap to an angle.
   */
  private final SwerveRequest.FieldCentricFacingAngle drivePerspectiveSnapsOpenLoop =
      new SwerveRequest.FieldCentricFacingAngle()
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
          .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
          .withDeadband(0.07)
          .withRotationalDeadband(0.5)
          .withHeadingPID(
              ORIGINAL_HEADING_PID.getP(), ORIGINAL_HEADING_PID.getI(), ORIGINAL_HEADING_PID.getD())
          .withMaxAbsRotationalRate(MAX_ANGULAR_RATE);

  /**
   * A {@link SwerveRequest} for use with {@link DriveSourceType#DRIVER_PERSPECTIVE_OPEN_LOOP}, but
   * overrides the angular velocity to instead snap to an angle.
   */
  private final SwerveRequest.FieldCentricFacingAngle drivePerspectiveIntakeSnapsOpenLoop =
      new SwerveRequest.FieldCentricFacingAngle()
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
          .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
          .withDeadband(0.07)
          .withRotationalDeadband(0.5)
          .withHeadingPID(
              ORIGINAL_HEADING_PID.getP(), ORIGINAL_HEADING_PID.getI(), ORIGINAL_HEADING_PID.getD())
          .withMaxAbsRotationalRate(MAX_ANGULAR_RATE);

  /** A {@link SwerveRequest} for use with {@link DriveSourceType#FIELD_CENTRIC_CLOSED_LOOP}. */
  private final SwerveRequest.FieldCentric fieldCentricClosedLoop =
      new SwerveRequest.FieldCentric()
          .withDriveRequestType(DriveRequestType.Velocity)
          .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);

  /**
   * A {@link SwerveRequest} for use with {@link DriveSourceType#FIELD_CENTRIC_CLOSED_LOOP}, but
   * overrides the angular velocity to instead snap to an angle.
   */
  private final SwerveRequest.FieldCentricFacingAngle fieldCentricSnapsClosedLoop =
      new SwerveRequest.FieldCentricFacingAngle()
          .withDriveRequestType(DriveRequestType.Velocity)
          .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
          .withDeadband(0.07)
          .withRotationalDeadband(0.5)
          .withHeadingPID(
              ORIGINAL_HEADING_PID.getP(),
              ORIGINAL_HEADING_PID.getI(),
              ORIGINAL_HEADING_PID.getD());

  /**
   * A {@link SwerveRequest} for use with {@link DriveSourceType#FIELD_CENTRIC_CLOSED_LOOP}, but
   * overrides the angular velocity to instead snap to an angle.
   */
  private final SwerveRequest.FieldCentricFacingAngle fieldCentricIntakeSnapsClosedLoop =
      new SwerveRequest.FieldCentricFacingAngle()
          .withDriveRequestType(DriveRequestType.Velocity)
          .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
          .withDeadband(0.07)
          .withRotationalDeadband(0.5)
          .withHeadingPID(
              ORIGINAL_HEADING_PID.getP(),
              ORIGINAL_HEADING_PID.getI(),
              ORIGINAL_HEADING_PID.getD());

  private final HealthManager health;

  private double lastSimTime;
  private @Nullable Notifier simNotifier = null;

  private SwerveDriveState drivetrainState = new SwerveDriveState();
  private ChassisSpeeds robotRelativeSpeeds = new ChassisSpeeds();
  private ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds();
  private ChassisSpeeds rateLimitedSpeeds = new ChassisSpeeds();

  private double turretStuckAimingAngle = 0.0;

  private boolean ableToBumpAssist = false;
  private boolean ableToTrenchAssist = false;
  private boolean ableToWallIntakeDriveAssist = false;
  private boolean ableToDirectionSnap = false;
  private Translation2d lastWallIntakePoint = Translation2d.kZero;
  private double distanceToWallIntakePoint = 0.0;
  private Rotation2d filteredLastDriveDirection = Rotation2d.kZero;

  public Swerve(
      TunerSwerveDrivetrain drivetrain,
      HealthManager health,
      XboxController driverController,
      Trailblazer trailblazer) {
    super(SubsystemPriority.SWERVE, SwerveState.MANUAL);
    this.drivetrain = drivetrain;
    this.health = health;

    if (Utils.isSimulation()) {
      startSimThread();
    }

    drivetrain.setStateStdDevs(new Matrix<>(VecBuilder.fill(0.003, 0.003, 0.002)));

    this.teleopDriveSource =
        new XboxControllerDriveSource(
            driverController, Swerve.MAX_LINEAR_RATE, Swerve.TELEOP_MAX_ANGULAR_RATE);
    this.trailblazerDriveSource =
        new TrailblazerDriveSource(
            trailblazer, () -> drivetrainState.Pose, this::getFieldRelativeSpeeds);
    driveSource = teleopDriveSource;
  }

  /**
   * Set the {@link DriveSource} to use. For some states, this might be ignored (ex. a state where
   * we are always using Trailblazer speeds instead of teleop).
   *
   * <p>This is meant to be called by the autos subsystem periodically, to ensure auto speeds are
   * being applied at the correct times.
   */
  public void setDriveSourceType(DriveSourceType type) {
    this.driveSource =
        switch (type) {
          case DRIVER_PERSPECTIVE_OPEN_LOOP -> teleopDriveSource;
          case FIELD_CENTRIC_CLOSED_LOOP -> trailblazerDriveSource;
        };
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return robotRelativeSpeeds;
  }

  public ChassisSpeeds getFieldRelativeSpeeds() {
    return fieldRelativeSpeeds;
  }

  public void climbAssistDriveRequest() {
    setStateFromRequest(SwerveState.CLIMB_ASSIST);
  }

  public void rateLimitedDriveRequest() {
    setStateFromRequest(SwerveState.MANUAL_RATE_LIMITED);
  }

  public void intakeRateLimitedDriveRequest() {
    setStateFromRequest(SwerveState.INTAKE_RATE_LIMITED);
  }

  public void turretStuckAimRequest(double snapAngle) {
    turretStuckAimingAngle = snapAngle;
    setStateFromRequest(SwerveState.TURRET_STUCK_SCORE);
  }

  @Override
  protected void collectInputs() {
    drivetrainState = drivetrain.getState();
    robotRelativeSpeeds = drivetrainState.Speeds;
    fieldRelativeSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(
            robotRelativeSpeeds, drivetrainState.Pose.getRotation());

    ableToTrenchAssist =
        DSOptions.USE_SWERVE_ASSIST.get()
            && driveSource.getDriveSourceType() == DriveSourceType.DRIVER_PERSPECTIVE_OPEN_LOOP
            && health.isLocalizationHealthy()
            && SwerveAssist.ableToTrenchAssist(drivetrainState.Pose, fieldRelativeSpeeds);
    ableToBumpAssist =
        DSOptions.USE_SWERVE_ASSIST.get()
            && driveSource.getDriveSourceType() == DriveSourceType.DRIVER_PERSPECTIVE_OPEN_LOOP
            && health.isLocalizationHealthy()
            && SwerveAssist.ableToBumpAssist(drivetrainState.Pose, fieldRelativeSpeeds);
    ableToWallIntakeDriveAssist =
        DSOptions.USE_SWERVE_ASSIST.get()
            && FeatureFlags.WALL_INTAKE_DRIVE_ASSIST.getAsBoolean()
            && driveSource.getDriveSourceType() == DriveSourceType.DRIVER_PERSPECTIVE_OPEN_LOOP
            && health.isLocalizationHealthy()
            && SwerveAssist.ableToWallIntakeDriveAssist(drivetrainState.Pose, fieldRelativeSpeeds);

    if (getState() == SwerveState.INTAKE) {
      lastWallIntakePoint =
          MathHelpers.getIntersectionOnRectanglePerimeter(
              drivetrainState.Pose.getTranslation(),
              FieldUtil.FIELD_BOUNDS,
              filteredLastDriveDirection);
      distanceToWallIntakePoint =
          lastWallIntakePoint.getDistance(drivetrainState.Pose.getTranslation());

      filteredLastDriveDirection =
          Rotation2d.fromDegrees(
              lastDriveDirectionFilter.calculate(
                  MathHelpers.getDriveDirection(fieldRelativeSpeeds).getDegrees()));

      ableToDirectionSnap =
          FeatureFlags.INTAKE_DIRECTIONAL_SNAPS.getAsBoolean()
              && driveSource.getDriveSourceType() == DriveSourceType.DRIVER_PERSPECTIVE_OPEN_LOOP
              && SwerveAssist.ableToDirectionSnap(fieldRelativeSpeeds);
    }

    var requestedSpeeds = driveSource.getRequestedSpeeds();
    if (FeatureFlags.RATE_LIMITED_DRIVING.getAsBoolean()
        && (getState() == SwerveState.INTAKE_RATE_LIMITED
            || getState() == SwerveState.MANUAL_RATE_LIMITED)) {

      if (driveSource.getDriveSourceType() == DriveSourceType.DRIVER_PERSPECTIVE_OPEN_LOOP) {
        var rateLimitedXVelocity =
            scoringXLinearVelocitySlewRateLimiter.calculate(requestedSpeeds.vxMetersPerSecond);
        var rateLimitedYVelocity =
            scoringYLinearVelocitySlewRateLimiter.calculate(requestedSpeeds.vyMetersPerSecond);
        var rateLimitedRotationalRate =
            MathUtil.clamp(
                requestedSpeeds.omegaRadiansPerSecond,
                -MAX_ANGULAR_RATE_SHOOTING,
                MAX_ANGULAR_RATE_SHOOTING);

        rateLimitedSpeeds =
            new ChassisSpeeds(
                rateLimitedXVelocity, rateLimitedYVelocity, rateLimitedRotationalRate);
      } else {
        rateLimitedSpeeds = requestedSpeeds;
      }
    } else {
      rateLimitedSpeeds = requestedSpeeds;
    }
  }

  public void normalDriveRequest() {
    setStateFromRequest(SwerveState.MANUAL);
  }

  public void intakeDriveRequest() {
    setStateFromRequest(SwerveState.INTAKE);
  }

  private SwerveRequest.FieldCentricFacingAngle withFieldRelativeTargetDirection(
      SwerveRequest.FieldCentricFacingAngle request, Rotation2d targetDirection) {
    if (request.ForwardPerspective == ForwardPerspectiveValue.OperatorPerspective) {
      var snapSetpoint =
          FmsUtil.isRedAlliance()
              ? targetDirection.plus(drivetrain.getOperatorForwardDirection())
              : targetDirection;

      return request.withTargetDirection(snapSetpoint);
    }

    return request.withTargetDirection(targetDirection);
  }

  @Override
  public void whileInState(SwerveState currentState) {
    drivetrain.setOperatorPerspectiveForward(
        FmsUtil.isRedAlliance() ? Rotation2d.k180deg : Rotation2d.kZero);

    switch (currentState) {
      case MANUAL -> {
        var speeds = driveSource.getRequestedSpeeds();
        if (ableToTrenchAssist) {

          DogLog.timestamp("Swerve/TrenchAssistActive");
          var trenchAssistSpeeds =
              SwerveAssist.getTrenchAssistSpeeds(drivetrainState.Pose.getTranslation(), speeds);
          drivetrain.setControl(
              withFieldRelativeTargetDirection(
                  drivePerspectiveSnapsOpenLoop
                      .withVelocityX(trenchAssistSpeeds.vxMetersPerSecond)
                      .withVelocityY(trenchAssistSpeeds.vyMetersPerSecond),
                  SwerveAssist.getRoundedSnapAngle(
                      drivetrainState.Pose.getRotation(), SwerveAssist.TRENCH_SNAP_ROUND_ANGLE)));
        } else if (ableToBumpAssist) {
          drivetrain.setControl(
              withFieldRelativeTargetDirection(
                  drivePerspectiveSnapsOpenLoop
                      .withVelocityX(speeds.vxMetersPerSecond)
                      .withVelocityY(speeds.vyMetersPerSecond),
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
      case MANUAL_RATE_LIMITED -> {
        if (ableToTrenchAssist) {

          DogLog.timestamp("TrenchAssistActive");
          var trenchAssistSpeeds =
              SwerveAssist.getTrenchAssistSpeeds(
                  drivetrainState.Pose.getTranslation(), rateLimitedSpeeds);
          drivetrain.setControl(
              withFieldRelativeTargetDirection(
                  drivePerspectiveSnapsOpenLoop
                      .withVelocityX(trenchAssistSpeeds.vxMetersPerSecond)
                      .withVelocityY(trenchAssistSpeeds.vyMetersPerSecond),
                  SwerveAssist.getRoundedSnapAngle(
                      drivetrainState.Pose.getRotation(), SwerveAssist.TRENCH_SNAP_ROUND_ANGLE)));
        } else if (ableToBumpAssist) {
          drivetrain.setControl(
              withFieldRelativeTargetDirection(
                  drivePerspectiveSnapsOpenLoop
                      .withVelocityX(rateLimitedSpeeds.vxMetersPerSecond)
                      .withVelocityY(rateLimitedSpeeds.vyMetersPerSecond),
                  SwerveAssist.getRoundedSnapAngle(
                      drivetrainState.Pose.getRotation(), SwerveAssist.BUMP_SNAP_ROUND_ANGLE)));
        } else if (driveSource.getDriveSourceType()
            == DriveSourceType.DRIVER_PERSPECTIVE_OPEN_LOOP) {

          drivetrain.setControl(
              driverPerspectiveOpenLoop
                  .withVelocityX(rateLimitedSpeeds.vxMetersPerSecond)
                  .withVelocityY(rateLimitedSpeeds.vyMetersPerSecond)
                  .withRotationalRate(rateLimitedSpeeds.omegaRadiansPerSecond));
        } else {

          drivetrain.setControl(
              fieldCentricClosedLoop
                  .withVelocityX(rateLimitedSpeeds.vxMetersPerSecond)
                  .withVelocityY(rateLimitedSpeeds.vyMetersPerSecond)
                  .withRotationalRate(rateLimitedSpeeds.omegaRadiansPerSecond));
        }
      }
      case INTAKE -> {
        var speeds = driveSource.getRequestedSpeeds();
        if (ableToTrenchAssist) {
          DogLog.timestamp("Swerve/TrenchAssistActive");
          var trenchAssistSpeeds =
              SwerveAssist.getTrenchAssistSpeeds(drivetrainState.Pose.getTranslation(), speeds);
          drivetrain.setControl(
              withFieldRelativeTargetDirection(
                  drivePerspectiveSnapsOpenLoop
                      .withVelocityX(trenchAssistSpeeds.vxMetersPerSecond)
                      .withVelocityY(trenchAssistSpeeds.vyMetersPerSecond),
                  SwerveAssist.getRoundedSnapAngle(
                      drivetrainState.Pose.getRotation(), SwerveAssist.TRENCH_SNAP_ROUND_ANGLE)));
        } else if (ableToBumpAssist) {
          drivetrain.setControl(
              withFieldRelativeTargetDirection(
                  drivePerspectiveSnapsOpenLoop
                      .withVelocityX(speeds.vxMetersPerSecond)
                      .withVelocityY(speeds.vyMetersPerSecond),
                  SwerveAssist.getRoundedSnapAngle(
                      drivetrainState.Pose.getRotation(), SwerveAssist.BUMP_SNAP_ROUND_ANGLE)));
        } else if (ableToDirectionSnap) {
          DogLog.timestamp("Swerve/DirectionSnaps/Snapping");
          var swerveSnapsRequest =
              driveSource.getDriveSourceType() == DriveSourceType.DRIVER_PERSPECTIVE_OPEN_LOOP
                  ? drivePerspectiveIntakeSnapsOpenLoop
                  : fieldCentricIntakeSnapsClosedLoop;
          drivetrain.setControl(
              withFieldRelativeTargetDirection(
                  swerveSnapsRequest
                      .withVelocityX(speeds.vxMetersPerSecond)
                      .withVelocityY(speeds.vyMetersPerSecond)
                      .withCenterOfRotation(Translation2d.kZero),
                  filteredLastDriveDirection));
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
      case INTAKE_RATE_LIMITED -> {
        if (ableToTrenchAssist) {
          DogLog.timestamp("TrenchAssistActive");
          var trenchAssistSpeeds =
              SwerveAssist.getTrenchAssistSpeeds(
                  drivetrainState.Pose.getTranslation(), rateLimitedSpeeds);
          drivetrain.setControl(
              withFieldRelativeTargetDirection(
                  drivePerspectiveSnapsOpenLoop
                      .withVelocityX(trenchAssistSpeeds.vxMetersPerSecond)
                      .withVelocityY(trenchAssistSpeeds.vyMetersPerSecond),
                  SwerveAssist.getRoundedSnapAngle(
                      drivetrainState.Pose.getRotation(), SwerveAssist.TRENCH_SNAP_ROUND_ANGLE)));
        } else if (ableToBumpAssist) {
          drivetrain.setControl(
              withFieldRelativeTargetDirection(
                  drivePerspectiveSnapsOpenLoop
                      .withVelocityX(rateLimitedSpeeds.vxMetersPerSecond)
                      .withVelocityY(rateLimitedSpeeds.vyMetersPerSecond),
                  SwerveAssist.getRoundedSnapAngle(
                      drivetrainState.Pose.getRotation(), SwerveAssist.BUMP_SNAP_ROUND_ANGLE)));
        } else if (ableToDirectionSnap) {
          DogLog.timestamp("Swerve/DirectionSnaps/Snapping");
          var swerveSnapsRequest =
              driveSource.getDriveSourceType() == DriveSourceType.DRIVER_PERSPECTIVE_OPEN_LOOP
                  ? drivePerspectiveIntakeSnapsOpenLoop
                  : fieldCentricIntakeSnapsClosedLoop;
          drivetrain.setControl(
              withFieldRelativeTargetDirection(
                  swerveSnapsRequest
                      .withVelocityX(rateLimitedSpeeds.vxMetersPerSecond)
                      .withVelocityY(rateLimitedSpeeds.vyMetersPerSecond)
                      .withCenterOfRotation(Translation2d.kZero),
                  filteredLastDriveDirection));
        } else {
          var swerveRequest =
              driveSource.getDriveSourceType() == DriveSourceType.DRIVER_PERSPECTIVE_OPEN_LOOP
                  ? driverPerspectiveOpenLoop
                  : fieldCentricClosedLoop;

          drivetrain.setControl(
              swerveRequest
                  .withVelocityX(rateLimitedSpeeds.vxMetersPerSecond)
                  .withVelocityY(rateLimitedSpeeds.vyMetersPerSecond)
                  .withRotationalRate(rateLimitedSpeeds.omegaRadiansPerSecond));
        }
      }
      case TURRET_STUCK_SCORE -> {
        var speeds = driveSource.getRequestedSpeeds();

        if (driveSource.getDriveSourceType() == DriveSourceType.DRIVER_PERSPECTIVE_OPEN_LOOP) {
          drivetrain.setControl(
              withFieldRelativeTargetDirection(
                  drivePerspectiveIntakeSnapsOpenLoop
                      .withVelocityX(speeds.vxMetersPerSecond)
                      .withVelocityY(speeds.vyMetersPerSecond),
                  Rotation2d.fromDegrees(turretStuckAimingAngle)));
        }
      }
      case CLIMB_ASSIST -> {
        // Always use Trailblazer drive source for climb alignment
        var speeds = trailblazerDriveSource.getRequestedSpeeds();

        drivetrain.setControl(
            fieldCentricClosedLoop
                .withVelocityX(speeds.vxMetersPerSecond)
                .withVelocityY(speeds.vyMetersPerSecond)
                .withRotationalRate(speeds.omegaRadiansPerSecond));
      }
    }

    DogLog.log("Swerve/ModuleStates", drivetrainState.ModuleStates);
    DogLog.log("Swerve/ModuleTargets", drivetrainState.ModuleTargets);
    DogLog.log(
        "SwerveSwerveTargetDirection", drivePerspectiveSnapsOpenLoop.TargetDirection.getDegrees());
    DogLog.log("Swerve/RobotRelativeSpeeds", drivetrainState.Speeds);
    DogLog.log("Swerve/FieldRelativeSpeeds", fieldRelativeSpeeds);
    DogLog.log("Swerve/AbleToBumpAssist", ableToBumpAssist);
    DogLog.log("Swerve/AbleToTrenchAssist", ableToTrenchAssist);
    DogLog.log("Swerve/AbleToWallIntakeDriveAssist", ableToWallIntakeDriveAssist);
  }

  @Override
  protected void beforeTransition(SwerveState fromState, SwerveState toState) {
    if (FeatureFlags.RATE_LIMITED_DRIVING.getAsBoolean()
        && (toState == SwerveState.INTAKE_RATE_LIMITED
            || toState == SwerveState.MANUAL_RATE_LIMITED)) {
      var requestedSpeeds = driveSource.getRequestedSpeeds();
      if (driveSource.getDriveSourceType() == DriveSourceType.DRIVER_PERSPECTIVE_OPEN_LOOP) {
        scoringXLinearVelocitySlewRateLimiter.reset(requestedSpeeds.vxMetersPerSecond);
        scoringYLinearVelocitySlewRateLimiter.reset(requestedSpeeds.vyMetersPerSecond);
      }
    }
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
