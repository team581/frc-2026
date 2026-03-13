package com.team581.mechanisms.imu;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.team581.math.MathHelpers;
import com.team581.util.scheduling.SubsystemPriorityBase;
import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;

public class BaseImuSubsystem extends StateMachineSubsystem<ImuState> {
  private static final double IS_FLAT_THRESHOLD = 5.0;

  protected final SwerveDrivetrain<?, ?, ?> drivetrain;
  private final Debouncer isFlatDebouncer = new Debouncer(0.5, DebounceType.kRising);

  private boolean isFlatDebounced = false;

  protected SwerveDriveState driveState = new SwerveDriveState();
  protected double robotHeading = 0;
  protected double robotAngularVelocity = 0;
  protected double pitch = 0;
  protected double roll = 0;

  public BaseImuSubsystem(SubsystemPriorityBase priority, SwerveDrivetrain<?, ?, ?> drivetrain) {
    super(priority, ImuState.DEFAULT_STATE);

    this.drivetrain = drivetrain;
  }

  public double getRobotAngularVelocity() {
    return robotAngularVelocity;
  }

  public double getRobotHeading() {
    return robotHeading;
  }

  public boolean isFlatDebounced() {
    return isFlatDebounced;
  }

  @Override
  public void whileInState(ImuState currentState) {
    DogLog.log("Imu/RobotHeading", robotHeading, Degrees);
    DogLog.log("Imu/AngularVelocity", robotAngularVelocity, DegreesPerSecond);
    DogLog.log("Imu/Pitch", pitch, Degrees);
    DogLog.log("Imu/Roll", roll, Degrees);
    DogLog.log("Imu/IsFlatDebounced", isFlatDebounced);
  }

  @Override
  protected void collectInputs() {
    driveState = drivetrain.getState();
    robotHeading = MathHelpers.angleModulus(driveState.Pose.getRotation().getDegrees());
    robotAngularVelocity = Math.toDegrees(driveState.Speeds.omegaRadiansPerSecond);

    pitch = drivetrain.getPigeon2().getPitch().getValueAsDouble();
    roll = drivetrain.getPigeon2().getRoll().getValueAsDouble();

    isFlatDebounced =
        isFlatDebouncer.calculate(
            MathUtil.isNear(pitch, 0, IS_FLAT_THRESHOLD, -90, 90)
                && MathUtil.isNear(roll, 0, IS_FLAT_THRESHOLD, -180, 180));
  }
}
