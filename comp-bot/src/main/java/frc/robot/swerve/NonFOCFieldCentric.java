package frc.robot.swerve;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.ModuleRequest;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * Field-centric swerve request that disables FOC on drive motors.
 *
 * <p>This is a reimplementation of {@link SwerveRequest.FieldCentric} that uses the non-native
 * {@link SwerveModule#apply(ModuleRequest)} path, allowing FOC to be explicitly disabled.
 */
public class NonFOCFieldCentric implements SwerveRequest {
  public double VelocityX = 0;
  public double VelocityY = 0;
  public double RotationalRate = 0;
  public double Deadband = 0;
  public double RotationalDeadband = 0;
  public Translation2d CenterOfRotation = new Translation2d();
  public DriveRequestType DriveRequestType = SwerveModule.DriveRequestType.OpenLoopVoltage;
  public SteerRequestType SteerRequestType = SwerveModule.SteerRequestType.Position;
  public boolean DesaturateWheelSpeeds = true;
  public ForwardPerspectiveValue ForwardPerspective = ForwardPerspectiveValue.OperatorPerspective;

  private final ModuleRequest moduleRequest = new ModuleRequest().withEnableFOC(false);

  @Override
  public StatusCode apply(
      SwerveControlParameters parameters, SwerveModule<?, ?, ?>... modulesToApply) {
    double toApplyX = VelocityX;
    double toApplyY = VelocityY;
    double toApplyOmega = RotationalRate;

    double linearMagnitude = Math.sqrt(toApplyX * toApplyX + toApplyY * toApplyY);
    if (linearMagnitude < Deadband) {
      toApplyX = 0;
      toApplyY = 0;
    }
    if (Math.abs(toApplyOmega) < RotationalDeadband) {
      toApplyOmega = 0;
    }

    Rotation2d robotAngle = parameters.currentPose.getRotation();
    if (ForwardPerspective == ForwardPerspectiveValue.OperatorPerspective) {
      robotAngle = robotAngle.minus(parameters.operatorForwardDirection);
    }

    ChassisSpeeds robotCentricSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(toApplyX, toApplyY, toApplyOmega, robotAngle);

    var states = parameters.kinematics.toSwerveModuleStates(robotCentricSpeeds, CenterOfRotation);
    if (DesaturateWheelSpeeds) {
      SwerveDriveKinematics.desaturateWheelSpeeds(states, parameters.kMaxSpeedMps);
    }

    moduleRequest
        .withDriveRequest(DriveRequestType)
        .withSteerRequest(SteerRequestType)
        .withUpdatePeriod(parameters.updatePeriod);

    for (int i = 0; i < modulesToApply.length; i++) {
      modulesToApply[i].apply(moduleRequest.withState(states[i]));
    }

    return StatusCode.OK;
  }

  public NonFOCFieldCentric withCenterOfRotation(Translation2d newCenterOfRotation) {
    this.CenterOfRotation = newCenterOfRotation;
    return this;
  }

  public NonFOCFieldCentric withDeadband(double newDeadband) {
    this.Deadband = newDeadband;
    return this;
  }

  public NonFOCFieldCentric withDesaturateWheelSpeeds(boolean newDesaturateWheelSpeeds) {
    this.DesaturateWheelSpeeds = newDesaturateWheelSpeeds;
    return this;
  }

  public NonFOCFieldCentric withDriveRequestType(DriveRequestType newDriveRequestType) {
    this.DriveRequestType = newDriveRequestType;
    return this;
  }

  public NonFOCFieldCentric withForwardPerspective(ForwardPerspectiveValue newForwardPerspective) {
    this.ForwardPerspective = newForwardPerspective;
    return this;
  }

  public NonFOCFieldCentric withRotationalDeadband(double newRotationalDeadband) {
    this.RotationalDeadband = newRotationalDeadband;
    return this;
  }

  public NonFOCFieldCentric withRotationalRate(double newRotationalRate) {
    this.RotationalRate = newRotationalRate;
    return this;
  }

  public NonFOCFieldCentric withSteerRequestType(SteerRequestType newSteerRequestType) {
    this.SteerRequestType = newSteerRequestType;
    return this;
  }

  public NonFOCFieldCentric withVelocityX(double newVelocityX) {
    this.VelocityX = newVelocityX;
    return this;
  }

  public NonFOCFieldCentric withVelocityY(double newVelocityY) {
    this.VelocityY = newVelocityY;
    return this;
  }
}
