package frc.robot.swerve;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.team581.math.CircularFilter;
import com.team581.math.MathHelpers;
import com.team581.mechanisms.PowerManaged;
import com.team581.swerve.DriveSource;
import com.team581.swerve.DriveSourceType;
import com.team581.swerve.SwerveAssist;
import com.team581.swerve.TrailblazerDriveSource;
import com.team581.swerve.XboxControllerDriveSource;
import com.team581.trailblazer.Trailblazer;
import com.team581.util.FmsUtil;
import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.Debouncer;
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
import frc.robot.generated.CompTunerConstants;
import frc.robot.generated.CompTunerConstants.TunerSwerveDrivetrain;
import frc.robot.health.HealthManager;
import frc.robot.util.AimParameterUtil.AimingParameters;
import frc.robot.util.scheduling.SubsystemPriority;
import org.jspecify.annotations.Nullable;

@SuppressWarnings("unused")
public class Swerve extends StateMachineSubsystem<SwerveState> implements PowerManaged {

  private static final DoubleSubscriber DRIVER_WANTS_SOTM_DELAY =
      DogLog.tunable("Swerve/DriverWantsSotmDelay", 0.3);
  ;

  public static final double TRANSLATION_STD_DEV = 0.01;

  public static final double MAX_LINEAR_RATE = 4.75;

  private boolean useLooseTolerance = false;
  private static final double LOOSE_TOLERANCE = 45.0;

  private static final DoubleSubscriber MAX_LINEAR_RATE_SHOOTING =
      DogLog.tunable("Swerve/MaxLinearRateMovingShot", 2.0);

  private static final DoubleSubscriber MAX_ANGULAR_RATE_SHOOTING =
      DogLog.tunable("Swerve/MaxAngularRateShootingRot", 4.0);
  private static final double MAX_ANGULAR_RATE = Units.rotationsToRadians(4.0);
  public static final Rotation2d TELEOP_MAX_ANGULAR_RATE = Rotation2d.fromRotations(2);

  private static final double SIM_LOOP_PERIOD = Units.millisecondsToSeconds(5);

  private final CircularFilter lastDriveDirectionFilter = new CircularFilter(1);
  private final SlewRateLimiter maxLinearVelocityRateLimiter = new SlewRateLimiter(5.0);
  private final SlewRateLimiter maxAngularVelocityRateLimiter = new SlewRateLimiter(5.0);

  public static final PhoenixPIDController ORIGINAL_HEADING_PID =
      new PhoenixPIDController(7.5, 0, 0);

  public final TunerSwerveDrivetrain drivetrain;

  private final XboxControllerDriveSource teleopDriveSource;
  private final TrailblazerDriveSource trailblazerDriveSource;
  private DriveSource driveSource;

  /** A {@link SwerveRequest} for use with {@link DriveSourceType#DRIVER_PERSPECTIVE_OPEN_LOOP}. */
  private final SwerveRequest.FieldCentric driverPerspective =
      new SwerveRequest.FieldCentric()
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
          .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
          .withDeadband(0.07)
          .withRotationalDeadband(0.05);

  /**
   * A {@link SwerveRequest} for use with {@link DriveSourceType#DRIVER_PERSPECTIVE_OPEN_LOOP}, but
   * overrides the angular velocity to instead snap to an angle.
   */
  private final SwerveRequest.FieldCentricFacingAngle drivePerspectiveSnaps =
      new SwerveRequest.FieldCentricFacingAngle()
          .withDriveRequestType(DriveRequestType.Velocity)
          .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
          .withDeadband(0.07)
          .withRotationalDeadband(0.0)
          .withHeadingPID(
              ORIGINAL_HEADING_PID.getP(), ORIGINAL_HEADING_PID.getI(), ORIGINAL_HEADING_PID.getD())
          .withMaxAbsRotationalRate(MAX_ANGULAR_RATE);

  /** A {@link SwerveRequest} for use with {@link DriveSourceType#FIELD_CENTRIC_CLOSED_LOOP}. */
  private final SwerveRequest.FieldCentric fieldCentric =
      new SwerveRequest.FieldCentric()
          .withDeadband(0.07)
          .withRotationalDeadband(0.05)
          .withDriveRequestType(DriveRequestType.Velocity)
          .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);

  /**
   * A {@link SwerveRequest} for use with {@link DriveSourceType#FIELD_CENTRIC_CLOSED_LOOP}, but
   * overrides the angular velocity to instead snap to an angle.
   */
  private final SwerveRequest.FieldCentricFacingAngle fieldCentricSnaps =
      new SwerveRequest.FieldCentricFacingAngle()
          .withDriveRequestType(DriveRequestType.Velocity)
          .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
          .withDeadband(0.07)
          .withRotationalDeadband(0.0)
          .withHeadingPID(
              ORIGINAL_HEADING_PID.getP(),
              ORIGINAL_HEADING_PID.getI(),
              ORIGINAL_HEADING_PID.getD());

  private final SwerveRequest.SwerveDriveBrake xRequest =
      new SwerveRequest.SwerveDriveBrake().withSteerRequestType(SteerRequestType.MotionMagicExpo);

  private final HealthManager health;

  private double lastSimTime;
  private @Nullable Notifier simNotifier = null;

  private SwerveDriveState drivetrainState = new SwerveDriveState();
  private ChassisSpeeds robotRelativeSpeeds = new ChassisSpeeds();
  private ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds();

  private double scoringAngle = 0.0;
  private double scoringTolerance = 0.0;
  private double scoringFeedForward = 0.0;

  private double feedingAngle = 0.0;
  private double feedingTolerance = 0.0;
  private double feedingFeedForward = 0.0;

  private double currentMaxLinearRate = MAX_LINEAR_RATE;
  private double currentMaxAngularRate = MAX_ANGULAR_RATE;

  private static final double MIN_AIMED_TOLERANCE = 2.0;

  private boolean ableToBumpAssist = false;
  private final Debouncer X_SWERVE_DEBOUNCER = new Debouncer(0.1);

  private boolean ableToXSwerve = false;

  private boolean driverWantsSotm = false;

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

  public ChassisSpeeds getRequestedSpeeds() {
    return driveSource.getRequestedSpeeds();
  }

  public void climbAssistDriveRequest() {
    setStateFromRequest(SwerveState.CLIMB_ASSIST);
  }

  public void scoreRequest(AimingParameters scoringParameters) {
    scoringAngle = scoringParameters.goalAngle();
    scoringTolerance =
        MathUtil.clamp(
            scoringParameters.swerveTolerance(), MIN_AIMED_TOLERANCE, Double.POSITIVE_INFINITY);
    scoringFeedForward = scoringParameters.swerveFeedForwardRadians();
    setStateFromRequest(SwerveState.SCORE);
  }

  public void warmupScoreRequest(AimingParameters scoringParameters) {
    scoringAngle = scoringParameters.goalAngle();
    scoringTolerance =
        MathUtil.clamp(
            scoringParameters.swerveTolerance(), MIN_AIMED_TOLERANCE, Double.POSITIVE_INFINITY);
    scoringFeedForward = scoringParameters.swerveFeedForwardRadians();
    setStateFromRequest(SwerveState.WARMUP_SCORE);
  }

  public void feedRequest(AimingParameters feedingParameters) {
    feedingAngle = feedingParameters.goalAngle();
    feedingTolerance =
        MathUtil.clamp(
            feedingParameters.swerveTolerance(), MIN_AIMED_TOLERANCE, Double.POSITIVE_INFINITY);
    feedingFeedForward = feedingParameters.swerveFeedForwardRadians();
    setStateFromRequest(SwerveState.FEED);
  }

  public void warmupFeedRequest(AimingParameters feedingParameters) {
    feedingAngle = feedingParameters.goalAngle();
    feedingTolerance =
        MathUtil.clamp(
            feedingParameters.swerveTolerance(), MIN_AIMED_TOLERANCE, Double.POSITIVE_INFINITY);
    feedingFeedForward = feedingParameters.swerveFeedForwardRadians();
    setStateFromRequest(SwerveState.WARMUP_FEED);
  }

  private double getTargetAngleDegrees() {
    return switch (getState()) {
      case WARMUP_SCORE, SCORE -> scoringAngle;
      case WARMUP_FEED, FEED -> feedingAngle;
      default -> 0.0;
    };
  }

  private double getToleranceDegrees() {
    return switch (getState()) {
      case WARMUP_SCORE, SCORE -> scoringTolerance;
      case WARMUP_FEED, FEED -> useLooseTolerance ? LOOSE_TOLERANCE : feedingTolerance;
      default -> 0.0;
    };
  }

  public boolean isMovingBeyondSafeSpeed() {
    var currentSpeeds = getFieldRelativeSpeeds();

    return MathHelpers.getLinearVelocity(currentSpeeds) > currentMaxLinearRate;
  }

  public void setUseLooseTolerance(boolean value) {
    useLooseTolerance = value;
  }

  public boolean atGoal(double lookaheadTime) {
    switch (getState()) {
      case WARMUP_SCORE, SCORE, WARMUP_FEED, FEED -> {}
      default -> {
        return atGoal();
      }
    }

    Rotation2d currentRotation = drivetrainState.Pose.getRotation();
    double angularVelocity = drivetrainState.Speeds.omegaRadiansPerSecond; // Radians per second

    var targetAngleDegrees = getTargetAngleDegrees();
    var toleranceDegrees = getToleranceDegrees();

    Rotation2d targetRotation = Rotation2d.fromDegrees(targetAngleDegrees);

    // Calculate the shortest signed angular distance from current to target
    double deltaAngleToGoal = targetRotation.minus(currentRotation).getRadians(); // Radians

    // If already within tolerance, we are at the goal
    if (MathUtil.isNear(0, deltaAngleToGoal, Math.toRadians(toleranceDegrees))) {
      return true;
    }

    // Check if we are moving towards the goal and will reach or pass it
    // Moving towards if the signs of angularVelocity and deltaAngleToGoal are the same
    // (or if one is zero, but deltaAngleToGoal is not zero here)
    if (Math.signum(angularVelocity) == Math.signum(deltaAngleToGoal)) {
      // Calculate the magnitude of rotation in lookahead time
      double rotationMagnitudeInLookahead = Math.abs(angularVelocity * lookaheadTime);

      // We reach or pass if the rotation magnitude is greater than or equal to the remaining
      // distance
      // (minus tolerance, since we just need to get within tolerance of the goal)
      return rotationMagnitudeInLookahead
          >= (Math.abs(deltaAngleToGoal) - Math.toRadians(toleranceDegrees));
    }

    // If not at goal, not moving towards it, or not reaching it, then not at or past
    return false;
  }

  public boolean atGoal() {
    return switch (getState()) {
      case WARMUP_SCORE, SCORE ->
          MathUtil.isNear(
              drivetrainState.Pose.getRotation().getDegrees(),
              scoringAngle,
              scoringTolerance,
              -180.0,
              180.0);
      case WARMUP_FEED, FEED ->
          MathUtil.isNear(
              drivetrainState.Pose.getRotation().getDegrees(),
              feedingAngle,
              useLooseTolerance ? LOOSE_TOLERANCE : feedingTolerance,
              -180,
              180);
      default -> false;
    };
  }

  public boolean isNear(double angle, double tolerance) {
    return MathUtil.isNear(
        drivetrainState.Pose.getRotation().getDegrees(), angle, tolerance, -180.0, 180.0);
  }

  @Override
  protected void collectInputs() {
    drivetrainState = drivetrain.getState();
    robotRelativeSpeeds = drivetrainState.Speeds;
    fieldRelativeSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(
            robotRelativeSpeeds, drivetrainState.Pose.getRotation());

    ableToXSwerve =
        switch (getState()) {
          case SCORE -> {
            yield X_SWERVE_DEBOUNCER.calculate(
                MathUtil.isNear(
                        Math.toDegrees(MathUtil.angleModulus(Math.toRadians(scoringAngle))),
                        drivetrainState.Pose.getRotation().getDegrees(),
                        scoringTolerance,
                        -180.0,
                        180.0)
                    && MathHelpers.getLinearVelocity(driveSource.getRequestedSpeeds()) < 1e-5);
          }
          case FEED -> {
            yield X_SWERVE_DEBOUNCER.calculate(
                MathUtil.isNear(
                        Math.toDegrees(MathUtil.angleModulus(Math.toRadians(feedingAngle))),
                        drivetrainState.Pose.getRotation().getDegrees(),
                        feedingTolerance,
                        -180.0,
                        180.0)
                    && MathHelpers.getLinearVelocity(driveSource.getRequestedSpeeds()) < 1e-5);
          }
          default -> false;
        };

    ableToBumpAssist =
        DSOptions.USE_BUMP_ASSIST.get()
            && driveSource.getDriveSourceType() == DriveSourceType.DRIVER_PERSPECTIVE_OPEN_LOOP
            && health.isLocalizationHealthy()
            && SwerveAssist.ableToBumpAssist(drivetrainState.Pose, fieldRelativeSpeeds);

    var currentDriveDirection =
        MathHelpers.getDriveDirection(driveSource.getRequestedSpeeds())
            .plus(Rotation2d.fromDegrees(FmsUtil.isRedAlliance() ? 180 : 0));

    var requestedSpeeds = driveSource.getRequestedSpeeds();
    driverWantsSotm =
        (timeout(DRIVER_WANTS_SOTM_DELAY.get())
                && (MathHelpers.getLinearVelocity(requestedSpeeds) > 1e-6
                    && (getState() == SwerveState.SCORE || getState() == SwerveState.FEED)))
            || driveSource.getDriveSourceType() != DriveSourceType.DRIVER_PERSPECTIVE_OPEN_LOOP;

    switch (getState()) {
      case SCORE, FEED -> {
        currentMaxAngularRate =
            maxAngularVelocityRateLimiter.calculate(MAX_ANGULAR_RATE_SHOOTING.get());
        currentMaxLinearRate =
            maxLinearVelocityRateLimiter.calculate(MAX_LINEAR_RATE_SHOOTING.get());
      }
      default -> {
        currentMaxAngularRate =
            maxAngularVelocityRateLimiter.calculate(Units.radiansToRotations(MAX_ANGULAR_RATE));
        currentMaxLinearRate = maxLinearVelocityRateLimiter.calculate(MAX_LINEAR_RATE);
      }
    }

    teleopDriveSource.setMaxVelocity(
        currentMaxLinearRate, Rotation2d.fromRotations(currentMaxAngularRate));

    drivePerspectiveSnaps.withMaxAbsRotationalRate(Units.rotationsToRadians(currentMaxAngularRate));
    fieldCentricSnaps.withMaxAbsRotationalRate(Units.rotationsToRadians(currentMaxAngularRate));
  }

  public void normalDriveRequest() {
    setStateFromRequest(SwerveState.MANUAL);
  }

  public boolean driverWantsSotm() {
    return driverWantsSotm;
  }

  public boolean driverStillDecidingSotm() {
    var requestedSpeeds = driveSource.getRequestedSpeeds();
    return (!timeout(DRIVER_WANTS_SOTM_DELAY.get())
            && (MathHelpers.getLinearVelocity(requestedSpeeds) > 1e-6
                && (getState() == SwerveState.SCORE || getState() == SwerveState.FEED)))
        && driveSource.getDriveSourceType() == DriveSourceType.DRIVER_PERSPECTIVE_OPEN_LOOP;
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
        if (ableToBumpAssist) {
          drivetrain.setControl(
              withFieldRelativeTargetDirection(
                  drivePerspectiveSnaps
                      .withVelocityX(speeds.vxMetersPerSecond)
                      .withVelocityY(speeds.vyMetersPerSecond),
                  SwerveAssist.getRoundedSnapAngle(
                      drivetrainState.Pose.getRotation(), SwerveAssist.BUMP_SNAP_ROUND_ANGLE)));
        } else {
          var swerveRequest =
              switch (driveSource.getDriveSourceType()) {
                case DRIVER_PERSPECTIVE_OPEN_LOOP -> driverPerspective;
                case FIELD_CENTRIC_CLOSED_LOOP -> fieldCentric;
              };

          drivetrain.setControl(
              swerveRequest
                  .withVelocityX(speeds.vxMetersPerSecond)
                  .withVelocityY(speeds.vyMetersPerSecond)
                  .withRotationalRate(speeds.omegaRadiansPerSecond));
        }
      }
      case WARMUP_SCORE, SCORE -> {
        var speeds = driveSource.getRequestedSpeeds();

        if (ableToXSwerve) {
          DogLog.timestamp("Swerve/XSwerveActive");
          drivetrain.setControl(xRequest);
        } else {
          switch (driveSource.getDriveSourceType()) {
            case DRIVER_PERSPECTIVE_OPEN_LOOP -> {
              if (!MathUtil.isNear(
                  0, driveSource.getRequestedSpeeds().omegaRadiansPerSecond, 1e-5)) {
                DogLog.timestamp("Swerve/DriverOverridingRotation");
                drivetrain.setControl(
                    driverPerspective
                        .withVelocityX(speeds.vxMetersPerSecond)
                        .withVelocityY(speeds.vyMetersPerSecond)
                        .withRotationalRate(speeds.omegaRadiansPerSecond));
              } else {
                DogLog.timestamp("Swerve/TryingToAim");
                drivetrain.setControl(
                    withFieldRelativeTargetDirection(
                            drivePerspectiveSnaps
                                .withVelocityX(speeds.vxMetersPerSecond)
                                .withVelocityY(speeds.vyMetersPerSecond),
                            Rotation2d.fromDegrees(scoringAngle))
                        .withTargetRateFeedforward(scoringFeedForward));
              }
            }
            case FIELD_CENTRIC_CLOSED_LOOP -> {
              DogLog.timestamp("Swerve/TryingToAim");
              drivetrain.setControl(
                  fieldCentricSnaps
                      .withVelocityX(speeds.vxMetersPerSecond)
                      .withVelocityY(speeds.vyMetersPerSecond)
                      .withTargetDirection(Rotation2d.fromDegrees(scoringAngle))
                      .withTargetRateFeedforward(scoringFeedForward));
            }
          }
        }
      }
      case WARMUP_FEED, FEED -> {
        var speeds = driveSource.getRequestedSpeeds();

        if (ableToXSwerve) {
          DogLog.timestamp("Swerve/XSwerveActive");
          drivetrain.setControl(xRequest);
        } else {
          switch (driveSource.getDriveSourceType()) {
            case DRIVER_PERSPECTIVE_OPEN_LOOP -> {
              if (!MathUtil.isNear(
                  0, driveSource.getRequestedSpeeds().omegaRadiansPerSecond, 1e-5)) {
                DogLog.timestamp("Swerve/DriverOverridingRotation");
                drivetrain.setControl(
                    driverPerspective
                        .withVelocityX(speeds.vxMetersPerSecond)
                        .withVelocityY(speeds.vyMetersPerSecond)
                        .withRotationalRate(speeds.omegaRadiansPerSecond));
              } else {
                DogLog.timestamp("Swerve/TryingToAim");
                drivetrain.setControl(
                    withFieldRelativeTargetDirection(
                            drivePerspectiveSnaps
                                .withVelocityX(speeds.vxMetersPerSecond)
                                .withVelocityY(speeds.vyMetersPerSecond),
                            Rotation2d.fromDegrees(feedingAngle))
                        .withTargetRateFeedforward(feedingFeedForward));
              }
            }
            case FIELD_CENTRIC_CLOSED_LOOP -> {
              DogLog.timestamp("Swerve/TryingToAim");
              drivetrain.setControl(
                  fieldCentricSnaps
                      .withVelocityX(speeds.vxMetersPerSecond)
                      .withVelocityY(speeds.vyMetersPerSecond)
                      .withTargetDirection(Rotation2d.fromDegrees(feedingAngle))
                      .withTargetRateFeedforward(feedingFeedForward));
            }
          }
        }
      }
      case CLIMB_ASSIST -> {
        // Always use Trailblazer drive source for climb alignment
        var speeds = trailblazerDriveSource.getRequestedSpeeds();

        drivetrain.setControl(
            fieldCentric
                .withVelocityX(speeds.vxMetersPerSecond)
                .withVelocityY(speeds.vyMetersPerSecond)
                .withRotationalRate(speeds.omegaRadiansPerSecond));
      }
    }

    if (DriverStation.isAutonomous() && DriverStation.isDisabled()) {
      var current = drivetrainState.ModuleStates;
      var targets = drivetrainState.ModuleTargets;
      boolean isMisaligned = false;

      for (int i = 0; i < current.length; i++) {
        var actual = current[i];
        var target = targets[i];

        if (MathUtil.isNear(actual.angle.getDegrees(), target.angle.getDegrees(), 5, -180, 180)) {
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
    } else {
      DogLog.clearFault("Swerve modules not pointed straight");
    }

    DogLog.log("Swerve/ModuleStates", drivetrainState.ModuleStates);
    DogLog.log("Swerve/ModuleTargets", drivetrainState.ModuleTargets);
    DogLog.log("Swerve/ScoringAngle", scoringAngle);
    DogLog.log("Swerve/FeedingAngle", feedingAngle);
    DogLog.log("Swerve/ClampedScoringTolerance", scoringTolerance);
    DogLog.log("Swerve/ClampedFeedingTolerance", feedingTolerance);
    DogLog.log("Swerve/SwerveTargetDirection", drivePerspectiveSnaps.TargetDirection.getDegrees());
    DogLog.log("Swerve/RobotRelativeSpeeds", drivetrainState.Speeds);
    DogLog.log("Swerve/FieldRelativeSpeeds", fieldRelativeSpeeds);
    DogLog.log("Swerve/AbleToBumpAssist", ableToBumpAssist);
    DogLog.log("Swerve/AbleToXSwerve", ableToXSwerve);
    DogLog.log(
        "Swerve/AbleToXSwerve/SpeedsNear0",
        MathHelpers.getLinearVelocity(driveSource.getRequestedSpeeds()) < 1e-5);

    DogLog.log("Swerve/DriverWantsSOTM", driverWantsSotm);
    DogLog.log("Swerve/DriverStillDecidingSotm", driverStillDecidingSotm());
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

  @Override
  public void applyCurrentLimits(double supplyCurrentLimit) {
    drivetrain
        .getModule(0)
        .getDriveMotor()
        .getConfigurator()
        .apply(
            CompTunerConstants.FrontLeft.DriveMotorInitialConfigs.CurrentLimits
                .withSupplyCurrentLimit(supplyCurrentLimit));
    drivetrain
        .getModule(1)
        .getDriveMotor()
        .getConfigurator()
        .apply(
            CompTunerConstants.FrontRight.DriveMotorInitialConfigs.CurrentLimits
                .withSupplyCurrentLimit(supplyCurrentLimit));
    drivetrain
        .getModule(2)
        .getDriveMotor()
        .getConfigurator()
        .apply(
            CompTunerConstants.BackLeft.DriveMotorInitialConfigs.CurrentLimits
                .withSupplyCurrentLimit(supplyCurrentLimit));

    drivetrain
        .getModule(3)
        .getDriveMotor()
        .getConfigurator()
        .apply(
            CompTunerConstants.BackRight.DriveMotorInitialConfigs.CurrentLimits
                .withSupplyCurrentLimit(supplyCurrentLimit));
  }
}
