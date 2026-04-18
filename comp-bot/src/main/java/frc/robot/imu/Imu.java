package frc.robot.imu;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.team581.mechanisms.imu.BaseImuSubsystem;
import com.team581.mechanisms.imu.BumpCrossingTracker;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.DoubleSubscriber;
import frc.robot.util.scheduling.SubsystemPriority;

public class Imu extends BaseImuSubsystem {
  private static final DoubleSubscriber COLLISION_G_FORCE_THRESHOLD =
      DogLog.tunable("Imu/CollisionGForceThreshold", 0.8);
  private final LinearFilter pigeonXAccelFilter = LinearFilter.movingAverage(10);

  private final LinearFilter pigeonYAccelFilter = LinearFilter.movingAverage(10);

  private final DoubleSubscriber MAX_ACCELRATION_THRESHOLD_SHOOTING =
      DogLog.tunable("Imu/MaxAccelerationThresholdShooting", 0.35);

  public final BumpCrossingTracker bumpCrossingTracker;

  private double pigeonXPrevAccel = 0.0;
  private double pigeonYPrevAccel = 0.0;
  private double pigeonGForce = 0.0;
  private double maxGForceDetected = Double.NEGATIVE_INFINITY;
  private double lastUpdateTime = 0.0;

  private double pigeonFilteredXAccel = 0.0;
  private double pigeonFilteredYAccel = 0.0;
  private double accel = 0.0;

  public Imu(SwerveDrivetrain<?, ?, ?> drivetrain) {
    super(SubsystemPriority.IMU, drivetrain);

    this.bumpCrossingTracker =
        new BumpCrossingTracker(
            () -> getPitch(),
            () -> getRoll(),
            translation ->
                drivetrain.resetPose(new Pose2d(translation, driveState.Pose.getRotation())));
  }

  public boolean accelerationLowEnoughToShoot() {
    return accel < MAX_ACCELRATION_THRESHOLD_SHOOTING.get();
  }

  @Override
  public void beforePeriodic() {
    super.beforePeriodic();
    bumpCrossingTracker.beforePeriodic();
  }

  @Override
  public void collectInputs() {
    super.collectInputs();

    pigeonFilteredXAccel =
        pigeonXAccelFilter.calculate(drivetrain.getPigeon2().getAccelerationX().getValueAsDouble());
    pigeonFilteredYAccel =
        pigeonYAccelFilter.calculate(drivetrain.getPigeon2().getAccelerationY().getValueAsDouble());
    accel = Math.hypot(pigeonFilteredXAccel, pigeonFilteredYAccel);

    // Calculates the jerk in the X and Y directions
    double currentTime = MathSharedStore.getTimestamp();
    double dt = lastUpdateTime == 0.0 ? 0.02 : currentTime - lastUpdateTime;
    lastUpdateTime = currentTime;

    double pigeonXJerk;
    double pigeonYJerk;
    if (dt == 0) {
      pigeonXJerk = 0.0;
      pigeonYJerk = 0.0;
    } else {
      pigeonXJerk = (pigeonFilteredXAccel - pigeonXPrevAccel) / dt;
      pigeonYJerk = (pigeonFilteredYAccel - pigeonYPrevAccel) / dt;
    }

    pigeonXPrevAccel = pigeonFilteredXAccel;
    pigeonYPrevAccel = pigeonFilteredYAccel;

    double pigeonJerk = Math.hypot(pigeonXJerk, pigeonYJerk);

    pigeonGForce = pigeonJerk / 9.81;
    maxGForceDetected = Math.max(maxGForceDetected, pigeonGForce);

    DogLog.log("Imu/Pigeon/XJerk", pigeonXJerk);
    DogLog.log("Imu/Pigeon/YJerk", pigeonYJerk);
    DogLog.log("Imu/Pigeon/HypotAccel", pigeonJerk);
    DogLog.log("Imu/Pigeon/GForce", pigeonGForce);
    DogLog.log("Imu/Pigeon/MaxGForce", maxGForceDetected);
    DogLog.log("Imu/Pigeon/FilteredAccel", accel);
    bumpCrossingTracker.log();
  }

  public boolean collisionDetected() {
    return pigeonGForce > COLLISION_G_FORCE_THRESHOLD.get();
  }

  @Override
  public void periodic() {
    super.periodic();
    bumpCrossingTracker.periodic();
  }
}
