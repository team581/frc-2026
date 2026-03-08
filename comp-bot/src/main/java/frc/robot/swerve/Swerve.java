package frc.robot.swerve;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.team581.autos.Point;
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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.config.DSOptions;
import frc.robot.config.FeatureFlags;
import frc.robot.generated.CompTunerConstants.TunerSwerveDrivetrain;
import frc.robot.health.HealthManager;
import frc.robot.turret.TurretConfig;
import frc.robot.util.scheduling.SubsystemPriority;
import org.jspecify.annotations.Nullable;

@SuppressWarnings("unused")
public class Swerve extends StateMachineSubsystem<SwerveState> {

  public static final double TRANSLATION_STD_DEV = 0.01;

  public static final double MAX_LINEAR_RATE = 4.75;
  private static final DoubleSubscriber MAX_LINEAR_RATE_SHOOTING =
      DogLog.tunable("Swerve/MaxLinearRateShooting", 2.0);

  private static final DoubleSubscriber SNAKE_MODE_AGRESSIVENESS =
      DogLog.tunable("Swerve/SnakeModeAgressiveness", 0.25);

  private static final double MAX_ANGULAR_RATE = Units.rotationsToRadians(4);
  private static final DoubleSubscriber MAX_ANGULAR_RATE_SHOOTING =
      DogLog.tunable("Swerve/MaxAngularRateShootingRot", 0.4);
  public static final Rotation2d TELEOP_MAX_ANGULAR_RATE = Rotation2d.fromRotations(2);

  private static final double SIM_LOOP_PERIOD = Units.millisecondsToSeconds(5);

  private final CircularFilter lastDriveDirectionFilter = new CircularFilter(1);
  private final SlewRateLimiter maxLinearVelocityRateLimiter = new SlewRateLimiter(5.0);
  private final SlewRateLimiter maxAngularVelocityRateLimiter = new SlewRateLimiter(5.0);

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
          .withRotationalDeadband(0.05)
          .withCenterOfRotation(TurretConfig.TURRET_TO_ROBOT.getTranslation());

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
          .withMaxAbsRotationalRate(MAX_ANGULAR_RATE)
          .withCenterOfRotation(TurretConfig.TURRET_TO_ROBOT.getTranslation());

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
          .withMaxAbsRotationalRate(MAX_ANGULAR_RATE)
          .withCenterOfRotation(TurretConfig.TURRET_TO_ROBOT.getTranslation());

  /** A {@link SwerveRequest} for use with {@link DriveSourceType#FIELD_CENTRIC_CLOSED_LOOP}. */
  private final SwerveRequest.FieldCentric fieldCentricClosedLoop =
      new SwerveRequest.FieldCentric()
          .withDriveRequestType(DriveRequestType.Velocity)
          .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
          .withCenterOfRotation(TurretConfig.TURRET_TO_ROBOT.getTranslation());

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
              ORIGINAL_HEADING_PID.getP(), ORIGINAL_HEADING_PID.getI(), ORIGINAL_HEADING_PID.getD())
          .withCenterOfRotation(TurretConfig.TURRET_TO_ROBOT.getTranslation());

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
              ORIGINAL_HEADING_PID.getP(), ORIGINAL_HEADING_PID.getI(), ORIGINAL_HEADING_PID.getD())
          .withCenterOfRotation(TurretConfig.TURRET_TO_ROBOT.getTranslation());

  private final HealthManager health;

  private double lastSimTime;
  private @Nullable Notifier simNotifier = null;

  private SwerveDriveState drivetrainState = new SwerveDriveState();
  private ChassisSpeeds robotRelativeSpeeds = new ChassisSpeeds();
  private ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds();

  private double turretStuckAimingAngle = 0.0;

  private boolean ableToBumpAssist = false;
  private boolean ableToTrenchAssist = false;
  private boolean ableToWallSnap = false;
  private boolean ableToDirectionSnap = false;
  private boolean inWallSnapCorner = false;
  private boolean previouslyInWallSnapCorner = false;
  private boolean enteredWallSnapCorner = false;
  private Rotation2d cornerSnapAngle = Rotation2d.kZero;
  private Rotation2d wallSnapAngle = Rotation2d.kZero;
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

    drivetrain.setStateStdDevs(
        new Matrix<>(VecBuilder.fill(TRANSLATION_STD_DEV, TRANSLATION_STD_DEV, 0.002)));

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
    ableToWallSnap =
        FeatureFlags.WALL_SNAPS.getAsBoolean()
            && !DriverStation.isAutonomous()
            && driveSource.getDriveSourceType() == DriveSourceType.DRIVER_PERSPECTIVE_OPEN_LOOP
            && health.isLocalizationHealthy()
            && SwerveAssist.ableToWallSnap(
                drivetrainState.Pose, fieldRelativeSpeeds, enteredWallSnapCorner);

    // Wall snap logic if we are in a corner
    inWallSnapCorner =
        Point.CLAMPED_POINTS_FEATURE_FLAG.getAsBoolean()
            ? FieldUtil.getCurrentHomeFieldWallSnapCornerZone(drivetrainState.Pose.getTranslation())
                .isPresent()
            : FieldUtil.getCurrentWallSnapCornerZone(drivetrainState.Pose.getTranslation())
                .isPresent();
    if (inWallSnapCorner
        && !previouslyInWallSnapCorner
        && SwerveAssist.drivingInWallSnapDirection(drivetrainState.Pose, fieldRelativeSpeeds)) {
      enteredWallSnapCorner = true;
    }
    if (!inWallSnapCorner) {
      enteredWallSnapCorner = false;
    }
    if (inWallSnapCorner && !previouslyInWallSnapCorner) {
      cornerSnapAngle =
          SwerveAssist.getWallSnapAngle(
              drivetrainState.Pose.getTranslation(), fieldRelativeSpeeds, inWallSnapCorner);
    }
    wallSnapAngle =
        enteredWallSnapCorner
            ? cornerSnapAngle
            : SwerveAssist.getWallSnapAngle(
                drivetrainState.Pose.getTranslation(), fieldRelativeSpeeds, false);
    previouslyInWallSnapCorner = inWallSnapCorner;

    if (getState() == SwerveState.INTAKE || getState() == SwerveState.INTAKE_RATE_LIMITED) {
      filteredLastDriveDirection =
          filteredLastDriveDirection.interpolate(
              MathHelpers.getDriveDirection(driveSource.getRequestedSpeeds())
                  .plus(Rotation2d.fromDegrees(FmsUtil.isRedAlliance() ? 180 : 0)),
              (SNAKE_MODE_AGRESSIVENESS.get()
                      * MathHelpers.getLinearVelocity(driveSource.getRequestedSpeeds()))
                  / teleopDriveSource.maxLinearVelocity);

      ableToDirectionSnap =
          FeatureFlags.INTAKE_DIRECTIONAL_SNAPS.getAsBoolean()
              && driveSource.getDriveSourceType() == DriveSourceType.DRIVER_PERSPECTIVE_OPEN_LOOP
              && SwerveAssist.ableToDirectionSnap(
                  fieldRelativeSpeeds, teleopDriveSource.getRightY());
    }

    var requestedSpeeds = driveSource.getRequestedSpeeds();
    if (getState() == SwerveState.INTAKE_RATE_LIMITED
        || getState() == SwerveState.MANUAL_RATE_LIMITED) {
      double maxAngularRateRotations =
          maxAngularVelocityRateLimiter.calculate(MAX_ANGULAR_RATE_SHOOTING.get());
      teleopDriveSource.setMaxVelocity(
          maxLinearVelocityRateLimiter.calculate(MAX_LINEAR_RATE_SHOOTING.get()),
          Rotation2d.fromRotations(maxAngularRateRotations));

      drivePerspectiveIntakeSnapsOpenLoop.withMaxAbsRotationalRate(
          Units.rotationsToRadians(maxAngularRateRotations));
      drivePerspectiveSnapsOpenLoop.withMaxAbsRotationalRate(
          Units.rotationsToRadians(maxAngularRateRotations));
      fieldCentricIntakeSnapsClosedLoop.withMaxAbsRotationalRate(
          Units.rotationsToRadians(maxAngularRateRotations));
      fieldCentricIntakeSnapsClosedLoop.withMaxAbsRotationalRate(
          Units.rotationsToRadians(maxAngularRateRotations));

    } else {
      double maxAngularRateRotations =
          maxAngularVelocityRateLimiter.calculate(TELEOP_MAX_ANGULAR_RATE.getRotations());
      teleopDriveSource.setMaxVelocity(
          maxLinearVelocityRateLimiter.calculate(MAX_LINEAR_RATE),
          Rotation2d.fromRotations(maxAngularRateRotations));
      drivePerspectiveIntakeSnapsOpenLoop.withMaxAbsRotationalRate(
          Units.rotationsToRadians(maxAngularRateRotations));
      drivePerspectiveSnapsOpenLoop.withMaxAbsRotationalRate(
          Units.rotationsToRadians(maxAngularRateRotations));
      fieldCentricIntakeSnapsClosedLoop.withMaxAbsRotationalRate(
          Units.rotationsToRadians(maxAngularRateRotations));
      fieldCentricIntakeSnapsClosedLoop.withMaxAbsRotationalRate(
          Units.rotationsToRadians(maxAngularRateRotations));
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
      case MANUAL, MANUAL_RATE_LIMITED -> {
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
      case INTAKE, INTAKE_RATE_LIMITED -> {
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
        } else if (ableToWallSnap) {
          drivetrain.setControl(
              withFieldRelativeTargetDirection(
                  drivePerspectiveSnapsOpenLoop
                      .withVelocityX(speeds.vxMetersPerSecond)
                      .withVelocityY(speeds.vyMetersPerSecond),
                  wallSnapAngle));
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
                      .withMaxAbsRotationalRate(
                          Units.rotationsToRadians(
                              maxAngularVelocityRateLimiter.calculate(
                                  MAX_ANGULAR_RATE_SHOOTING.get()))),
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

        if (DriverStation.isAutonomous() && DriverStation.isDisabled()) {
          var current = drivetrainState.ModuleStates;
          var targets = drivetrainState.ModuleTargets;
          boolean isMisaligned = false;

          for (int i = 0; i < current.length; i++) {
            var actual = current[i];
            var target = targets[i];

            if (MathUtil.isNear(
                actual.angle.getDegrees(), target.angle.getDegrees(), 5, -180, 180)) {
              // it's within tolerance
            } else {
              isMisaligned = true;
              break;
            }
          }

          if (isMisaligned) {
            DogLog.logFault("Swerve modules not pointed straight", AlertType.kError);
          } else {
            DogLog.clearFault("Swerve modules not pointed straight");
          }
        }
      }
    }

    DogLog.log("Swerve/ModuleStates", drivetrainState.ModuleStates);
    DogLog.log("Swerve/ModuleTargets", drivetrainState.ModuleTargets);
    DogLog.log(
        "Swerve/SwerveTargetDirection", drivePerspectiveSnapsOpenLoop.TargetDirection.getDegrees());
    DogLog.log("Swerve/RobotRelativeSpeeds", drivetrainState.Speeds);
    DogLog.log("Swerve/FieldRelativeSpeeds", fieldRelativeSpeeds);
    DogLog.log("Swerve/AbleToBumpAssist", ableToBumpAssist);
    DogLog.log("Swerve/AbleToTrenchAssist", ableToTrenchAssist);
    DogLog.log("Swerve/AbleToWallSnap", ableToWallSnap);
    DogLog.log(
        "SwerveAssist/WallSnaps/WallSnapAngle",
        SwerveAssist.getWallSnapAngle(
                drivetrainState.Pose.getTranslation(), fieldRelativeSpeeds, false)
            .getDegrees());
    DogLog.log("SwerveAssist/WallSnaps/CornerSnapAngle", cornerSnapAngle.getDegrees());
    DogLog.log("SwerveAssist/WallSnaps/ChosenAngle", wallSnapAngle.getDegrees());
  }

  @Override
  protected void afterTransition(SwerveState newState) {
    switch (newState) {
      case MANUAL, MANUAL_RATE_LIMITED, TURRET_STUCK_SCORE, CLIMB_ASSIST -> {}
      // When entering intake states we shouldnt be able to wall snap corner transition
      case INTAKE, INTAKE_RATE_LIMITED -> enteredWallSnapCorner = false;
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
