package frc.robot.swerve;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Field-centric swerve request with heading PID that disables FOC on drive motors.
 *
 * <p>This is a reimplementation of {@link SwerveRequest.FieldCentricFacingAngle} that delegates to
 * {@link NonFOCFieldCentric} instead of the native {@link SwerveRequest.FieldCentric}.
 */
public class NonFOCFieldCentricFacingAngle implements SwerveRequest {
  public double VelocityX = 0;
  public double VelocityY = 0;
  public Rotation2d TargetDirection = new Rotation2d();
  public double TargetRateFeedforward = 0;
  public double Deadband = 0;
  public double RotationalDeadband = 0;
  public double MaxAbsRotationalRate = 0;
  public Translation2d CenterOfRotation = new Translation2d();
  public DriveRequestType DriveRequestType = SwerveModule.DriveRequestType.OpenLoopVoltage;
  public SteerRequestType SteerRequestType = SwerveModule.SteerRequestType.Position;
  public boolean DesaturateWheelSpeeds = true;
  public ForwardPerspectiveValue ForwardPerspective = ForwardPerspectiveValue.OperatorPerspective;
  public PhoenixPIDController HeadingController = new PhoenixPIDController(0, 0, 0);

  private final NonFOCFieldCentric fieldCentric = new NonFOCFieldCentric();

  public NonFOCFieldCentricFacingAngle() {
    HeadingController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public StatusCode apply(
      SwerveControlParameters parameters, SwerveModule<?, ?, ?>... modulesToApply) {
    Rotation2d angleToFace = TargetDirection;
    if (ForwardPerspective == ForwardPerspectiveValue.OperatorPerspective) {
      angleToFace = angleToFace.rotateBy(parameters.operatorForwardDirection);
    }

    double toApplyOmega =
        TargetRateFeedforward
            + HeadingController.calculate(
                parameters.currentPose.getRotation().getRadians(),
                angleToFace.getRadians(),
                parameters.timestamp);
    if (MaxAbsRotationalRate > 0.0) {
      if (toApplyOmega > MaxAbsRotationalRate) {
        toApplyOmega = MaxAbsRotationalRate;
      } else if (toApplyOmega < -MaxAbsRotationalRate) {
        toApplyOmega = -MaxAbsRotationalRate;
      }
    }

    return fieldCentric
        .withVelocityX(VelocityX)
        .withVelocityY(VelocityY)
        .withRotationalRate(toApplyOmega)
        .withDeadband(Deadband)
        .withRotationalDeadband(RotationalDeadband)
        .withCenterOfRotation(CenterOfRotation)
        .withDriveRequestType(DriveRequestType)
        .withSteerRequestType(SteerRequestType)
        .withDesaturateWheelSpeeds(DesaturateWheelSpeeds)
        .withForwardPerspective(ForwardPerspective)
        .apply(parameters, modulesToApply);
  }

  public NonFOCFieldCentricFacingAngle withCenterOfRotation(Translation2d newCenterOfRotation) {
    this.CenterOfRotation = newCenterOfRotation;
    return this;
  }

  public NonFOCFieldCentricFacingAngle withDeadband(double newDeadband) {
    this.Deadband = newDeadband;
    return this;
  }

  public NonFOCFieldCentricFacingAngle withDesaturateWheelSpeeds(boolean newDesaturateWheelSpeeds) {
    this.DesaturateWheelSpeeds = newDesaturateWheelSpeeds;
    return this;
  }

  public NonFOCFieldCentricFacingAngle withDriveRequestType(DriveRequestType newDriveRequestType) {
    this.DriveRequestType = newDriveRequestType;
    return this;
  }

  public NonFOCFieldCentricFacingAngle withForwardPerspective(
      ForwardPerspectiveValue newForwardPerspective) {
    this.ForwardPerspective = newForwardPerspective;
    return this;
  }

  public NonFOCFieldCentricFacingAngle withHeadingPID(double kP, double kI, double kD) {
    this.HeadingController.setPID(kP, kI, kD);
    return this;
  }

  public NonFOCFieldCentricFacingAngle withMaxAbsRotationalRate(double newMaxAbsRotationalRate) {
    this.MaxAbsRotationalRate = newMaxAbsRotationalRate;
    return this;
  }

  public NonFOCFieldCentricFacingAngle withRotationalDeadband(double newRotationalDeadband) {
    this.RotationalDeadband = newRotationalDeadband;
    return this;
  }

  public NonFOCFieldCentricFacingAngle withSteerRequestType(SteerRequestType newSteerRequestType) {
    this.SteerRequestType = newSteerRequestType;
    return this;
  }

  public NonFOCFieldCentricFacingAngle withTargetDirection(Rotation2d newTargetDirection) {
    this.TargetDirection = newTargetDirection;
    return this;
  }

  public NonFOCFieldCentricFacingAngle withTargetRateFeedforward(double newTargetRateFeedforward) {
    this.TargetRateFeedforward = newTargetRateFeedforward;
    return this;
  }

  public NonFOCFieldCentricFacingAngle withVelocityX(double newVelocityX) {
    this.VelocityX = newVelocityX;
    return this;
  }

  public NonFOCFieldCentricFacingAngle withVelocityY(double newVelocityY) {
    this.VelocityY = newVelocityY;
    return this;
  }
}
